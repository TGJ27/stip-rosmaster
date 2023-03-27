#!/usr/bin/env python3
"""STIP dashboard HTTP API. Stays up when ROS master/adapter/stages fail."""
import argparse
import secrets
import hmac
import base64
import collections
import json
import os
import queue
import re
import signal
import shutil
import struct
import subprocess
import sys
import threading
import time
import urllib.parse
import uuid
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from storage import Repository, safe_name
from processes import Processes
from ros_bridge import ROSBridge
from map_images import pgm_to_png, map_meta
from inspection_gallery import list_inspection_runs, load_inspection_image
from qr_profiles import get_profile, profile_image
from fire_settings import read_thresholds, write_thresholds
from tracking_settings import read_tracking, save_tracking
from gas_alarm import GasAlarm
from fire_events import list_events, image as fire_event_image



class App:
    def __init__(self, args):
        self.args=args
        self.repos=Repository(args.maps_dir,args.data_dir)
        self.inspection_base=Path(os.environ.get('STIP_INSPECTION_DATA_DIR',str(Path.home()/'Documents'/'stip_inspection_data')))
        self.inspection_runs_dir=self.inspection_base/'runs'
        self.fire_event_dir=self.inspection_base/'fire_events'
        self.qr_profile_dir=self.inspection_base/'profiles'
        self.processes=Processes()
        self.bridge=ROSBridge()
        self.gas_alarm=GasAlarm(Path(args.data_dir)/'settings'/'gas_alarm.json')
        self.gas_command_sent=None
        self.gas_command_at=0.0
        self.lock=threading.RLock()
        self.engine=args.engine
        self.system_started=False
        self.logs=collections.deque(maxlen=100)
        self.power_enabled=True  # Actual execution requires passwordless sudo for the exact commands.
        self.active_nav_map=None
        self.token=os.environ.get('STIP_DASHBOARD_TOKEN','')
        self.session=secrets.token_urlsafe(40)
        # Local-network mode: no token unless STIP_DASHBOARD_TOKEN is explicitly set.
        threading.Thread(target=self.gas_alarm_loop,daemon=True).start()

    def gas_alarm_loop(self):
        """MQ-6 safety-adjacent indication runs even with no browser open."""
        while not self.bridge.stopping:
            try:
                ros=dict(self.bridge.state)
                edge=self.gas_alarm.evaluate(ros)
                status=self.gas_alarm.snapshot()
                active=status['active']
                hardware=ros.get('gas_alarm_state')
                eligible=status['status'] in ('NORMAL','ALARM')
                # A sensor failure never authorizes sending an ALARM OFF.
                # Hardware ACK represents combined FIRE or GAS; ACK=ON during fire
                # is not a gas alarm mismatch and must never trigger gas OFF spam.
                # Reassert a latched gas request even if its sensor goes offline.
                should_send=(edge is not None or (active and hardware is not True))
                if should_send and (edge is not None or time.monotonic()-self.gas_command_at>=4):
                    self.gas_command_at=time.monotonic()
                    try:
                        self.bridge.command('gas_alarm',{'enabled':active},timeout=2)
                        self.gas_command_sent=active
                        if edge is not None:
                            message='MQ-6 relative ADC alarm {}: {}% vs threshold {}%'.format(
                                'ON' if active else 'OFF',status['change_percent'],status['threshold_percent'])
                            self.logs.append(message)
                    except Exception as exc:
                        self.logs.append('MQ-6 alarm command pending/rejected: '+str(exc)[:140])
            except Exception as exc:
                self.logs.append('MQ-6 alarm monitor error: '+str(exc)[:140])
            time.sleep(0.4)


    def diagnostic(self):
        state=dict(self.bridge.state)
        if self.bridge.last_heard and time.monotonic()-self.bridge.last_heard>5:
            state={'connected':False,'error':'ROS heartbeat stale'}
        return {'ros':state,'gas_alarm':dict(self.gas_alarm.snapshot(),hardware_ack=state.get('gas_alarm_state')),'processes':self.processes.status(),
                'events':list(self.bridge.events)[-25:]+list(self.processes.logs)[-45:]+list(self.logs)[-25:],
                'engine':self.engine,'power_enabled':self.power_enabled, 'maps_directory':str(self.repos.maps_dir),
                'map':self.bridge.map_meta,'time':int(time.time())}

    def detector_config(self):
        r=subprocess.run(['rospack','find','stip_fire_detection'],capture_output=True,text=True,timeout=5)
        if r.returncode!=0:raise ValueError('stip_fire_detection ROS package not found')
        package=Path(r.stdout.strip()).resolve()
        config=package/'config'/'fire_cpp.yaml'
        if not config.is_file():
            raise ValueError('Native fire detector config not found')
        return config

    def update_fire_thresholds(self,payload):
        state=self.bridge.state
        ms=state.get('mission',{}).get('state')
        if ms in ('RUNNING','PAUSED') or state.get('fire_hold') or state.get('confirmed'):
            raise ValueError('Stop patrol and clear fire hold before changing detector settings')
        if not bool(payload.get('ack_restart')):
            raise ValueError('Confirm managed fire-detector restart')
        cfg=self.detector_config()
        changed=write_thresholds(cfg,payload['fire'],payload['smoke'])
        if not changed:return {'saved':True,'restart_queued':False,'values':read_thresholds(cfg)}
        try:
            result=self.bridge.command('stage_restart',{'stage':'fire'},timeout=6)
        except Exception as exc:
            raise ValueError('Thresholds saved, but fire restart NOT confirmed. Restart fire stage manually. '+str(exc))
        return {'saved':True,'restart_queued':True,'values':read_thresholds(cfg),'message':str(result)}

    def tracking_config(self):
        r=subprocess.run(['rospack','find','stip_fire_inspection'],capture_output=True,text=True,timeout=5)
        if r.returncode!=0:raise ValueError('stip_fire_inspection ROS package not available')
        path=Path(r.stdout.strip()).resolve()/'config'/'inspection.yaml'
        if not path.is_file():raise ValueError('Fire-inspection controller config missing')
        return path

    def update_tracking_speed(self,payload):
        state=self.bridge.state
        if state.get('mission',{}).get('state') in ('RUNNING','PAUSED') or state.get('fire_hold') or state.get('confirmed') or state.get('fire'):
            raise ValueError('Stop patrol, turn Fire Monitoring OFF and clear any confirmed fire before changing tracking speed')
        if not bool(payload.get('ack_restart')):
            raise ValueError('Confirm fire-inspection controller restart')
        result=save_tracking(self.tracking_config(),payload['step'],payload['interval'])
        if not result['changed']:
            return dict(result,applied=True,restart_required=False)
        # Restart only the dashboard-owned fire-inspection process; do not
        # restart the fire detector, thermal driver, stage manager, or other owners.
        if self.processes.status().get('fire_inspection',{}).get('running'):
            stopped=self.processes.stop('fire_inspection')
            if not stopped.get('stopped'):
                raise ValueError('Settings saved; fire-inspection stop unconfirmed. Restart the controller manually before use.')
            try:
                self.processes.start('fire_inspection',
                    ['roslaunch','stip_fire_inspection','fire_inspection.launch',
                     'auto_aim:=true','stationary_test_mode:=false','enable_pop32i_buzzer:=false'])
            except Exception as exc:
                raise ValueError('Settings saved; managed fire-inspection controller did not restart: '+str(exc))
            return dict(result,applied=False,restart_required=True,
                        message='Restart requested. Wait for fire inspection ready and verify actual ROS parameters.')
        return dict(result,applied=False,restart_required=True,
                    message='Settings saved. External ROS fire-inspection controller must be restarted to apply them.')

    def map_env(self):
        return os.environ.copy()

    def package_exists(self,name):
        try:
            r=subprocess.run(['rospack','find',name],capture_output=True,text=True,timeout=4)
            return r.returncode==0 and Path(r.stdout.strip()).is_dir()
        except Exception:return False

    def start_system(self):
        if self.processes.status().get('system',{}).get('running'):
            raise ValueError('System is already managed by this dashboard')
        # Existing external stage manager must not be duplicated.
        try:
            names=subprocess.run(['rosnode','list'],capture_output=True,text=True,timeout=3).stdout
        except Exception:names=''
        if '/stip_stage_manager' in names:
            raise ValueError('Existing stage manager detected. Attach to it; do not start a duplicate.')
        if '/driver_node' in names:
            raise ValueError('Robot driver already running outside manager: stop it before dashboard system startup')
        return self.processes.start('system',['roslaunch','stip_rosmaster_manager','dashboard_system.launch'],self.map_env())

    def start_nav_mode(self, kind, name=None):
        state=self.processes.status()
        other='navigation' if kind=='mapping' else 'mapping'
        if state.get(other,{}).get('running'):
            raise ValueError('Stop {} before starting {}'.format(other,kind))
        if state.get(kind,{}).get('running'):
            if kind=='navigation' and self.active_nav_map==name:
                return {'attached':True,'name':name}
            raise ValueError('{} already active'.format(kind))
        if not self.bridge.state.get('connected'):
            raise ValueError('ROS adapter offline. Start ROSMASTER system/bringup first.')
        if not self.bridge.state.get('modules',{}).get('bringup',{}).get('state')=='READY':
            raise ValueError('Robot bringup not READY. Start the existing stage manager first.')
        # The stage manager owns base/camera/sensors; LiDAR is separate and
        # needed by both SLAM and navigation. Only start it if no fresh /scan.
        if not self.bridge.state.get('health',{}).get('lidar',False):
            if not self.processes.status().get('lidar',{}).get('running'):
                self.processes.start('lidar', ['roslaunch','stip_rosmaster_nav','lidar.launch'],self.map_env())
            deadline=time.monotonic()+12
            while time.monotonic()<deadline:
                if self.bridge.state.get('health',{}).get('lidar',False):break
                if not self.processes.status().get('lidar',{}).get('running'):
                    raise ValueError('LiDAR launch exited; open Home logs')
                time.sleep(.2)
            else:
                raise ValueError('LiDAR not publishing /scan; inspect LiDAR launch and hardware')
        if kind=='navigation':
            safe_name(name)
            if not self.repos.map_exists(name):raise ValueError('Map not found')
        if self.engine=='yahboom':
            if not self.package_exists('yahboomcar_nav'):
                raise ValueError('yahboomcar_nav not installed; switch to STIP backend')
            if kind=='mapping':
                cmd=['roslaunch','yahboomcar_nav','yahboomcar_map.launch','use_rviz:=false','map_type:=gmapping']
            else:
                # Yahboom nav resolves maps relative to its own maps directory.
                source=self.repos.maps_dir
                rp=subprocess.run(['rospack','find','yahboomcar_nav'],capture_output=True,text=True,timeout=4)
                target=Path(rp.stdout.strip())/'maps'
                if not target.is_dir():raise ValueError('Yahboom maps directory is missing')
                for ext in ('.yaml','.pgm'):
                    src=source/(name+ext);dest=target/(name+ext)
                    if dest.exists() and dest.read_bytes()!=src.read_bytes():
                        raise ValueError('Vendor map name conflicts with existing file: '+str(dest))
                    if not dest.exists():shutil.copy2(src,dest)
                cmd=['roslaunch','yahboomcar_nav','yahboomcar_navigation.launch',
                     'use_rviz:=false','map:='+name]
        else:
            cmd=['roslaunch','stip_rosmaster_nav',
                 'mapping.launch' if kind=='mapping' else 'navigation.launch','use_rviz:=false']
            if kind=='navigation':cmd.append('map:='+name)
        result=self.processes.start(kind,cmd,self.map_env())
        if kind=='navigation':self.active_nav_map=name
        return result

    def save_map(self, name):
        name=safe_name(name)
        if self.repos.map_exists(name):raise ValueError('Map name already exists')
        if not self.processes.status().get('mapping',{}).get('running'):
            raise ValueError('SLAM mapping is not running')
        prefix=self.repos.maps_dir/('.tmp_'+uuid.uuid4().hex)
        cmd=['rosrun','map_server','map_saver','-f',str(prefix)]
        try:
            result=subprocess.run(cmd,capture_output=True,text=True,timeout=22)
            pgm=Path(str(prefix)+'.pgm');yaml=Path(str(prefix)+'.yaml')
            if result.returncode!=0 or not pgm.is_file() or not yaml.is_file():
                raise ValueError('map_saver failed: '+(result.stderr or result.stdout)[-400:])
            final_pgm=self.repos.maps_dir/(name+'.pgm')
            final_yaml=self.repos.maps_dir/(name+'.yaml')
            txt=yaml.read_text()
            txt=re.sub(r'(?m)^image:\s*.*$', 'image: '+name+'.pgm',txt)
            # Commit PNG-backed map pair without overwriting a name.
            os.link(str(pgm),str(final_pgm))
            try:
                fd=os.open(str(final_yaml),os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o644)
                with os.fdopen(fd,'w') as f:f.write(txt)
            except BaseException:
                final_pgm.unlink(missing_ok=True)
                raise
            return {'name':name,'saved':True}
        finally:
            for ext in ('.pgm','.yaml'):
                Path(str(prefix)+ext).unlink(missing_ok=True)

    def shutdown_robot(self, kind):
        # Never execute arbitrary shell commands or prompt sudo from the web server.
        command_name = 'reboot' if kind == 'restart' else 'poweroff' if kind == 'poweroff' else None
        if command_name is None:
            raise ValueError('Unknown power action')
        executable = '/usr/sbin/' + command_name
        if not Path(executable).is_file():
            raise ValueError('Power command missing: ' + executable)
        # Verify that sudo will not prompt BEFORE stopping managed ROS processes.
        check = subprocess.run(['sudo', '-n', '-l', executable],
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                               text=True, timeout=5)
        if check.returncode != 0:
            raise ValueError('Permission denied. Configure passwordless sudo for only ' + executable +
                             ' (see README_POWER_AND_MAPS.md); ' + check.stderr.strip()[:180])
        try:self.bridge.command('stop', timeout=3)
        except Exception:pass
        for name in ('navigation','mapping','fire_inspection','thermal_overlay','lidar','system'):
            try:self.processes.stop(name)
            except Exception:pass
        # Start asynchronously: reboot/poweroff will terminate the server shortly.
        child=subprocess.Popen(['sudo','-n',executable], stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL, start_new_session=True)
        return {'accepted':True,'pid':child.pid}

    def action(self, kind, payload):
        if kind=='start_system':return self.start_system()
        if kind=='stop_system':
            # Stop owned mission/navigation processes even when ROS adapter dies.
            try:self.bridge.command('stop',timeout=2)
            except Exception as exc:self.processes.logs.append('Software STOP unavailable: '+str(exc))
            result={}
            for component in ('navigation','mapping','fire_inspection','thermal_overlay','lidar','system'):
                result[component]=self.processes.stop(component)
            self.active_nav_map=None
            return {'stopped':all(v.get('stopped') for v in result.values()),'processes':result}
        if kind=='start_mapping':return self.start_nav_mode('mapping')
        if kind=='stop_mapping':
            try:self.bridge.command('stop',timeout=2)
            except Exception:pass
            return self.processes.stop('mapping')
        if kind=='save_map':return self.save_map(payload['name'])
        if kind=='delete_map':
            if any(self.processes.status().get(k,{}).get('running') for k in ('mapping','navigation')):
                raise ValueError('Stop mapping/navigation before deleting saved maps')
            return self.repos.delete_map(payload['name'])
        if kind=='start_navigation':return self.start_nav_mode('navigation',payload['map'])
        if kind=='stop_navigation':
            # Navigation teardown must work even if the ROS adapter is stuck.
            # Always attempt to stop the dashboard-owned navigation process.
            try:self.bridge.command('stop_navigation_only',timeout=2)
            except Exception as exc:self.processes.logs.append('Navigation stop ROS adapter unavailable: '+str(exc))
            self.active_nav_map=None
            return self.processes.stop('navigation')
        if kind=='exit_patrol':
            # A confirmed fire is not a normal Patrol-owned setting: retain the
            # ROS controller and its safety hold until operator acknowledgement.
            # In all other cases, exiting Patrol means fire monitoring AND
            # auto-tracking are OFF before Settings can open.
            bridge_error=None
            try:self.bridge.command('stop',timeout=2)
            except Exception as exc:
                bridge_error=str(exc)
                self.processes.logs.append('Patrol exit ROS STOP failed: '+bridge_error)
            nav_result=self.processes.stop('navigation')
            self.active_nav_map=None
            if not nav_result.get('stopped'):
                raise ValueError('Navigation could not be confirmed stopped; remain in Patrol and inspect the robot')
            if bridge_error:
                raise ValueError('Navigation stopped but ROS STOP was not acknowledged: '+bridge_error)
            ros_state=dict(getattr(self.bridge,'state',{}) or {})
            if ros_state.get('confirmed'):
                raise ValueError('Confirmed fire is still active. Robot stopped; acknowledge the confirmed fire in Patrol before exiting.')

            # Do not assume changing browser pages turns ROS services off.
            # Work against the actual ROS state and the processes we own.
            monitoring_on=bool(ros_state.get('fire'))
            owned_controller=bool(self.processes.status().get('fire_inspection',{}).get('running'))
            if monitoring_on or owned_controller:
                try:
                    result=self.bridge.command('fire_tracking',{'enabled':False},timeout=5)
                    if not result or result.get('enabled') is not False:
                        raise ValueError('Auto-tracking disable not acknowledged')
                except Exception as exc:
                    raise ValueError('Robot stopped, but fire auto-tracking did not confirm OFF: '+str(exc))
            if monitoring_on:
                try:
                    result=self.bridge.command('fire_enable',{'enabled':False},timeout=5)
                    if not result or result.get('enabled') is not False:
                        raise ValueError('Fire detector disable not acknowledged')
                    if result.get('fire_hold'):
                        raise ValueError('ROS fire hold remains active; inspect and acknowledge the fire controller')
                    # The ROS worker applies OFF before acknowledging this RPC.
                    # Mirror the acknowledged state immediately so Settings
                    # cannot reject the very next Save due to a stale 5-Hz
                    # bridge status snapshot.
                    self.bridge.state['fire']=False
                    self.bridge.state['fire_hold']=False
                except Exception as exc:
                    raise ValueError('Robot stopped, but Fire Monitoring did not confirm OFF: '+str(exc))
            if owned_controller:
                stopped=self.processes.stop('fire_inspection')
                if not stopped.get('stopped'):
                    raise ValueError('Fire Monitoring disabled, but dashboard-owned fire inspection did not stop')
            try:self.processes.stop('thermal_overlay')
            except Exception as exc:self.processes.logs.append('Overlay cleanup warning: '+str(exc))
            return {'stopped':True,'navigation_stopped':True,
                    'fire_monitoring_off':True,'tracking_off':True,
                    'fire_controller_stopped':bool(owned_controller)}
        if kind=='exit_mission':
            # Editor owns manual move_base goals and dashboard-owned navigation.
            # Do not claim a safe page exit if ROS STOP was not acknowledged.
            bridge_error=None
            try:self.bridge.command('stop',timeout=2)
            except Exception as exc:
                bridge_error=str(exc)
                self.processes.logs.append('Mission exit ROS STOP unavailable: '+bridge_error)
            result=self.processes.stop('navigation')
            self.active_nav_map=None
            if not result.get('stopped'):
                raise ValueError('Mission navigation stop not confirmed')
            if bridge_error:
                raise ValueError('Navigation stopped but ROS STOP was not acknowledged: '+bridge_error)
            return {'stopped':True,'navigation_stopped':True}
        if kind=='fire_thresholds_set':return self.update_fire_thresholds(payload)
        if kind=='tracking_speed_set':return self.update_tracking_speed(payload)
        if kind=='gas_alarm_settings_set':return self.gas_alarm.save(payload['threshold_percent'])
        if kind=='patrol_takeover':
            if not self.processes.status().get('navigation',{}).get('running'):
                raise ValueError('Navigation must be started to use Patrol manual drive')
            return self.bridge.command('patrol_takeover',{},timeout=5)
        if kind=='save_mission':return self.repos.save(payload['mission'])
        if kind=='delete_mission':
            self.repos.delete_mission(payload['id']);return {'deleted':True}
        if kind=='robot_system_restart':
            if self.bridge.state.get('confirmed') or self.bridge.state.get('fire_hold'):
                raise ValueError('Fire safety hold active: clear only after verifying the scene; restart refused')
            if self.bridge.state.get('mission',{}).get('state') in ('RUNNING','PAUSED'):
                raise ValueError('Stop the active Patrol before restarting robot subsystems')
            # Prevent revived move_base from resending a goal after driver restart.
            try:
                self.bridge.command('stop',timeout=3)  # full hardware restart revokes teleop sessions
            except Exception as exc:
                raise ValueError('Cannot confirm robot zero-velocity stop: '+str(exc))
            stopped={}
            for component in ('navigation','mapping','fire_inspection','thermal_overlay','lidar'):
                stopped[component]=self.processes.stop(component)
            if not all(x.get('stopped') for x in stopped.values()):
                raise ValueError('One or more dashboard-owned ROS stages could not be stopped; restart refused')
            self.active_nav_map=None
            result=self.bridge.command('robot_system_restart',{},timeout=6)
            return dict(result, navigation_stopped=True, dashboard_preserved=True)
        if kind=='stage_restart':
            if self.bridge.state.get('mission',{}).get('state') in ('RUNNING','PAUSED'):
                raise ValueError('Stop active patrol before restarting components')
            stage=payload.get('stage','')
            if stage not in ('bringup','camera','qr','fire','gas','thermal','ui'):
                raise ValueError('Unknown managed stage')
            return self.bridge.command('stage_restart',{'stage':stage})
        if kind=='power_restart':return self.shutdown_robot('restart')
        if kind=='power_off':return self.shutdown_robot('poweroff')
        if kind=='qr_test':
            return self.bridge.command(kind,payload,timeout=75)
        if kind=='patrol_start':
            mission=self.repos.get(payload['mission_id'])
            mission['loop']=bool(payload.get('loop',mission.get('loop',False)))
            if not self.processes.status().get('navigation',{}).get('running'):
                raise ValueError('Navigation stack is not running')
            return self.bridge.command('patrol_start',{'mission':mission, 'operator_confirmed':bool(payload.get('operator_confirmed',False))})
        if kind=='thermal_overlay_start':
            if self.processes.status().get('thermal_overlay',{}).get('running'):
                return {'running':True,'managed':True}
            try:
                nodes=subprocess.run(['rosnode','list'],capture_output=True,text=True,timeout=3)
                if nodes.returncode==0 and '/stip_thermal_rgb_overlay' in nodes.stdout.splitlines():
                    return {'running':True,'managed':False,'attached':True}
            except (OSError,subprocess.TimeoutExpired):pass
            cfg=subprocess.run(['rospack','find','stip_thermal_camera'],capture_output=True,text=True,timeout=5)
            if cfg.returncode!=0:raise ValueError('Thermal ROS package unavailable')
            alignment=Path(cfg.stdout.strip())/'config'/'thermal_rgb_alignment.yaml'
            if not alignment.is_file():raise ValueError('Missing saved thermal RGB alignment: '+str(alignment))
            # Reuse the working ROS node's calibration and registration pipeline.
            # Do not launch its camera/thermal parents or create duplicate drivers.
            return self.processes.start('thermal_overlay',
                ['rosrun','stip_thermal_camera','thermal_rgb_overlay.py',
                 '_show:=false','_display_fps:=10','_decode_scale:=2',
                 '_alpha:=0.45','_alignment_file:='+str(alignment)])
        if kind=='thermal_overlay_stop':
            # Stops only a dashboard-owned node; external calibrated overlay remains.
            return self.processes.stop('thermal_overlay')
        if kind=='fire_inspection_start':
            if self.processes.status().get('fire_inspection',{}).get('running'):
                return {'attached':True}
            return self.processes.start('fire_inspection',
                ['roslaunch','stip_fire_inspection','fire_inspection.launch',
                 'auto_aim:=true','stationary_test_mode:=false','enable_pop32i_buzzer:=false'])
        if kind=='teleop_arm' and bool(payload.get('enabled')):
            processes=self.processes.status()
            if not (processes.get('mapping',{}).get('running') or processes.get('navigation',{}).get('running')):
                raise ValueError('Start mapping or navigation before enabling teleop')
            if self.bridge.state.get('mission',{}).get('state') in ('RUNNING','PAUSED'):
                raise ValueError('Stop the active patrol before manual teleop')
        if kind=='goal' and self.bridge.state.get('mission',{}).get('state') in ('RUNNING','PAUSED'):
            raise ValueError('Manual navigation blocked while patrol owns move_base')
        return self.bridge.command(kind,payload,timeout=10 if kind in ('qr_enroll','qr_prepare','qr_capture') else 4)



class Handler(BaseHTTPRequestHandler):
    server_version='STIPDashboard/0.1'
    def log_message(self,fmt,*args):
        if not self.path.startswith('/api/status'):super().log_message(fmt,*args)

    def send_common(self,code,ctype='application/json',length=None):
        self.send_response(code)
        self.send_header('Content-Type',ctype)
        self.send_header('Cache-Control','no-store')
        self.send_header('X-Content-Type-Options','nosniff')
        self.send_header('X-Frame-Options','DENY')
        self.send_header('Referrer-Policy','no-referrer')
        if length is not None:self.send_header('Content-Length',str(length))
        self.end_headers()

    def output(self,obj,code=200):
        data=json.dumps(obj).encode()
        try:
            self.send_common(code,length=len(data))
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            # Browser refreshed/navigated away while this HTTP reply was pending.
            # No retry is appropriate for robot-control commands.
            return

    def protected(self):
        app=self.server.app
        if app.token:
            supplied=self.headers.get('X-Dashboard-Token','')
            cookie=self.headers.get('Cookie','')
            session=next((x.strip().split('=',1)[1] for x in cookie.split(';') if x.strip().startswith('stip_session=')), '')
            if not (hmac.compare_digest(supplied,app.token) or hmac.compare_digest(session,app.session)):return False
        return True

    def do_GET(self):
        app=self.server.app
        parsed=urllib.parse.urlsplit(self.path)
        url=parsed.path
        if url.startswith('/api/') and not self.protected():
            return self.output({'error':'Dashboard token required'},401)
        try:
            if url=='/api/session':
                data=b'{"ok":true}'
                self.send_response(200)
                self.send_header('Content-Type','application/json')
                self.send_header('Cache-Control','no-store')
                self.send_header('X-Content-Type-Options','nosniff')
                self.send_header('Set-Cookie','stip_session='+app.session+'; HttpOnly; SameSite=Strict; Path=/')
                self.send_header('Content-Length',str(len(data)))
                self.end_headers()
                return self.wfile.write(data)
            if url=='/api/status':return self.output(app.diagnostic())
            if url=='/api/maps':return self.output({'maps':app.repos.maps()})
            if url=='/api/missions':return self.output({'missions':app.repos.list_missions()})
            if url=='/api/fire_thresholds':return self.output({'values':read_thresholds(app.detector_config()),'requires_restart':True})
            if url=='/api/tracking_speed':return self.output({'values':read_tracking(app.tracking_config()),'requires_restart':True})
            if url=='/api/gas_alarm_settings':return self.output({'values':app.gas_alarm.snapshot(),'requires_restart':False})
            if url=='/api/fire_events':return self.output({'events':list_events(app.fire_event_dir)})
            evidence=re.fullmatch(r'/api/fire_events/(fire_[0-9]{8}_[0-9]{6}_[0-9a-f]{8})/(rgb|thermal)\.jpg',url)
            if evidence:
                data=fire_event_image(app.fire_event_dir,*evidence.groups())
                self.send_common(200,'image/jpeg',len(data));return self.wfile.write(data)
            qrdata=re.fullmatch(r'/api/qr_profiles/([A-Za-z0-9][A-Za-z0-9_-]{0,63})',url)
            if qrdata:return self.output({'profile':get_profile(app.qr_profile_dir,qrdata.group(1))})
            qrimage=re.fullmatch(r'/api/qr_profiles/([A-Za-z0-9][A-Za-z0-9_-]{0,63})/(reference|annotated)\.jpg',url)
            if qrimage:
                data=profile_image(app.qr_profile_dir,*qrimage.groups())
                self.send_common(200,'image/jpeg',len(data));return self.wfile.write(data)
            if url=='/api/inspection_runs':return self.output({'runs':list_inspection_runs(app.inspection_runs_dir)})
            gallery=re.fullmatch(r'/api/inspection_runs/([A-Za-z0-9_-]{1,100})/([A-Za-z0-9_-]{1,100})/(annotated|original|indicator)\.jpg',url)
            if gallery:
                data=load_inspection_image(app.inspection_runs_dir,*gallery.groups())
                self.send_common(200,'image/jpeg',len(data));return self.wfile.write(data)
            if url=='/api/live_map':
                data=app.bridge.live_map
                if not data:return self.output({'error':'No live /map received'},404)
                self.send_common(200,'image/png',len(data));return self.wfile.write(data)
            if url=='/api/thermal.mjpg':
                self.send_response(200)
                self.send_header('Content-Type','multipart/x-mixed-replace; boundary=stipframe')
                self.send_header('Cache-Control','no-cache, no-store, must-revalidate')
                self.send_header('X-Content-Type-Options','nosniff')
                self.end_headers()
                last=None
                try:
                    while not app.bridge.stopping:
                        data=app.bridge.thermal
                        if data and data is not last:
                            self.wfile.write(b'--stipframe\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(data)).encode()+b'\r\n\r\n')
                            self.wfile.write(data)
                            self.wfile.write(b'\r\n');self.wfile.flush()
                            last=data
                        time.sleep(.07)
                except (BrokenPipeError,ConnectionResetError,OSError):return
            if url=='/api/thermal':
                data=app.bridge.thermal
                if not data:return self.output({'error':'Thermal unavailable'},404)
                self.send_common(200,'image/jpeg',len(data));return self.wfile.write(data)
            if url=='/api/camera_registered.mjpg':
                # This is the existing calibrated ROS overlay node's BGR8 output,
                # converted to JPEG by the adapter. No dashboard-side stretching.
                self.send_response(200)
                self.send_header('Content-Type','multipart/x-mixed-replace; boundary=stipframe')
                self.send_header('Cache-Control','no-cache, no-store, must-revalidate')
                self.send_header('X-Content-Type-Options','nosniff')
                self.end_headers()
                last=None
                try:
                    while not app.bridge.stopping:
                        data=app.bridge.registered_camera
                        if data and data is not last:
                            self.wfile.write(b'--stipframe\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(data)).encode()+b'\r\n\r\n')
                            self.wfile.write(data)
                            self.wfile.write(b'\r\n')
                            self.wfile.flush()
                            last=data
                        time.sleep(.05)
                except (BrokenPipeError, ConnectionResetError, OSError):return
            if url=='/api/overlay_status':
                data=app.bridge.registered_camera
                owned=app.processes.status().get('thermal_overlay',{}).get('running',False)
                return self.output({'ready':bool(data),'managed':bool(owned),
                                    'source':'/thermal/rgb_overlay',
                                    'calibration':'stip_thermal_camera/config/thermal_rgb_alignment.yaml'})
            if url=='/api/camera.mjpg':
                self.send_response(200)
                self.send_header('Content-Type','multipart/x-mixed-replace; boundary=stipframe')
                self.send_header('Cache-Control','no-cache, no-store, must-revalidate')
                self.send_header('X-Content-Type-Options','nosniff')
                self.end_headers()
                last=None
                try:
                    while not app.bridge.stopping:
                        data=app.bridge.camera
                        if data and data is not last:
                            self.wfile.write(b'--stipframe\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(data)).encode()+b'\r\n\r\n')
                            self.wfile.write(data)
                            self.wfile.write(b'\r\n')
                            self.wfile.flush()
                            last=data
                        time.sleep(0.035)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    return
            if url=='/api/camera':
                data=app.bridge.camera
                if not data:return self.output({'error':'Camera unavailable'},404)
                self.send_common(200,'image/jpeg',len(data));return self.wfile.write(data)
            m=re.fullmatch(r'/api/maps/([A-Za-z0-9_-]{1,64})\.(png|meta)',url)
            if m:
                name,ext=m.groups()
                if not app.repos.map_exists(name):return self.output({'error':'Map not found'},404)
                if ext=='meta':return self.output(map_meta(app.repos.maps_dir/(name+'.yaml')))
                data=pgm_to_png(app.repos.maps_dir/(name+'.pgm'))
                self.send_common(200,'image/png',len(data));return self.wfile.write(data)
            m_static=re.fullmatch(r'/static/(style\.css|js/[a-z_]+\.js)',url)
            if m_static:
                rel=m_static.group(1)
                asset=HERE/'static'/rel
                if not asset.is_file():return self.output({'error':'Asset not found'},404)
                contents=asset.read_bytes()
                ctype='text/css; charset=utf-8' if rel.endswith('.css') else 'application/javascript; charset=utf-8'
                self.send_common(200,ctype,len(contents));return self.wfile.write(contents)
            if url=='/' or url=='/index.html':
                data=(HERE/'static'/'index.html').read_bytes()
                self.send_common(200,'text/html; charset=utf-8',len(data));return self.wfile.write(data)
            return self.output({'error':'Not found'},404)
        except Exception as exc:return self.output({'error':str(exc)},500)

    def do_POST(self):
        if not self.protected():return self.output({'error':'Dashboard token required'},401)
        if self.path!='/api/action':return self.output({'error':'Not found'},404)
        if self.headers.get('Content-Type','').split(';')[0]!='application/json':
            return self.output({'error':'JSON content type required'},415)
        if int(self.headers.get('Content-Length','0'))>1500000:return self.output({'error':'Body too large'},413)
        try:
            raw=self.rfile.read(int(self.headers.get('Content-Length','0')))
            payload=json.loads(raw)
            name=payload['action'];args=payload.get('payload',{})
            if not isinstance(args,dict):raise ValueError('Invalid payload')
            answer=self.server.app.action(name,args)
            return self.output({'ok':True,'result':answer})
        except (ValueError,KeyError,FileNotFoundError) as exc:return self.output({'ok':False,'error':str(exc)},400)
        except Exception as exc:return self.output({'ok':False,'error':str(exc)},500)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--host',default='127.0.0.1')
    ap.add_argument('--port',type=int,default=8765)
    ap.add_argument('--engine',choices=['stip','yahboom'],default=os.environ.get('STIP_NAV_BACKEND','stip'))
    ap.add_argument('--maps-dir',default=str(HERE.parents[1]/'stip_rosmaster_nav'/'maps'))
    ap.add_argument('--data-dir',default=str(Path.home()/'Documents'/'stip_dashboard_data'))
    args=ap.parse_args()
    app=App(args)
    server=ThreadingHTTPServer((args.host,args.port),Handler)
    server.daemon_threads=True
    server.app=app
    print('STIP dashboard at http://{}:{}/ — engine {}'.format(args.host,args.port,args.engine),flush=True)
    stopping={'requested':False}
    def graceful_signal(signum, frame):
        if stopping['requested']:return
        stopping['requested']=True
        # shutdown() must not be invoked on serve_forever's thread.
        threading.Thread(target=server.shutdown,daemon=True).start()
    signal.signal(signal.SIGTERM,graceful_signal)
    try:server.serve_forever(poll_interval=.25)
    except KeyboardInterrupt:pass
    finally:
        try:app.bridge.command('stop',timeout=.5)
        except Exception:pass
        app.bridge.stopping=True
        try:app.bridge.camera_path.unlink(missing_ok=True)
        except Exception:pass
        try:app.bridge.registered_path.unlink(missing_ok=True)
        except Exception:pass
        app.processes.stop_all()
        server.server_close()

if __name__=='__main__':main()
