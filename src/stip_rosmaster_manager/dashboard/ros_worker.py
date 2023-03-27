#!/usr/bin/env python3
"""ROS1 adapter subprocess. ROS failures cannot terminate HTTP server.
Protocol: newline-delimited JSON stdin/out. No stdout logging except events.
"""
import base64
import json
import math
import os
import queue
import threading
import time
import sys
from arm_hold import ArmHold
from camera_feed import write_frame
from pose_policy import usable_map_pose
from qr_decode import decode_qr
from inspection_geometry import parse_qr_geometry, parse_fire_detections
from fire_policy import evaluate_fire_hold
from fire_events import save_event
from pathlib import Path

import rospy
import tf
import actionlib
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import String, Bool, Float32, Int32, Float64MultiArray
from std_srvs.srv import SetBool, Trigger
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from stip_rosmaster_msgs.msg import ArmJoint
from stip_qr_inspection.srv import RunInspection

write_lock = threading.Lock()

def emit(kind, **kwargs):
    with write_lock:
        try:
            sys.stdout.write(json.dumps(dict(kind=kind, **kwargs), separators=(',', ':')) + '\n')
            sys.stdout.flush()
        except BrokenPipeError:
            os._exit(0)


class Adapter:
    def __init__(self):
        rospy.init_node('stip_dashboard_adapter', anonymous=True, disable_signals=True)
        self.lock = threading.RLock()
        self.state = {'connected': True, 'robot': {}, 'health': {}, 'nav': 'IDLE',
                      'mission': {'state': 'IDLE', 'index': 0}, 'arm': {'pan': 90, 'tilt': 0},
                      'fire': False, 'warning': False, 'confirmed': False, 'fire_hold': False, 'fire_hold_reason': '', 'fire_state': 'UNKNOWN', 'arrived_id': None,
                      'localization': {'ready': False, 'operator_confirmed': False}, 'gas': None,
                      'inspection_results': [], 'gas_calibration_status':'UNKNOWN'}
        self.cached_jpeg = None
        self.camera_frame_path=os.environ.get('STIP_DASHBOARD_CAMERA_PATH')
        self.registered_frame_path=os.environ.get('STIP_DASHBOARD_REGISTERED_PATH')
        self.last_registered=0.0
        self.last_arm_publishers_check=0.0
        self.frame_capture = {}
        self.fire_status_payload = {}
        self.fire_rgb_snapshot = None
        self.fire_thermal_snapshot = None
        self.fire_target_temp = None
        self.fire_background_temp = None
        self.fire_evidence_dir = Path.home()/'Documents'/'stip_inspection_data'/'fire_events'
        self.map_image = None
        self.last_cam = 0.0
        self.last_thermal=0.0
        self.qr_enabled=True
        self.last_map = 0.0
        self.scan_time = 0.0
        self.last_laser_send = 0.0
        self.last_tf_warning = 0.0
        self.last_motion_active = False
        self.tf_listener = tf.TransformListener()
        self.pose_command_time = 0.0
        self.amcl_samples = 0
        self.operator_confirmed=False
        self.last_amcl_time = 0.0
        self.mission = None
        self.patrol_thread = None
        self.patrol_halt = threading.Event()
        self.pause = threading.Event()
        self.goal_cancel = threading.Event()
        self.fire_hold = False
        self.fire_pause_requested = False
        self.teleop_armed = False
        # Serializes commands arriving on independent RPC handler threads.
        self.motion_lock = threading.RLock()
        self.teleop_session = 0  # Invalidates delayed packets after each arm/disarm.
        self.last_teleop = 0
        self.teleop_seq = -1
        self.arm_hold=ArmHold(self.state['arm'], interval=.18, timeout=.42, degrees_per_sec=8.0)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.gas_alarm_pub = rospy.Publisher('/gas_sensor/alarm_cmd', Bool, queue_size=1)
        self.pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=2)
        # Register only while this dashboard owns a manual camera-arm operation.
        # The ROS fire inspection package enforces exclusive /TargetAngle ownership.
        self.arm_pub = None
        self.last_arm_command = 0.0
        self.arm_pub_lock = threading.RLock()
        self.pause_ack = rospy.Publisher('/fire_inspection/navigation_paused', Bool, queue_size=1)
        rospy.Subscriber('/stip/startup/modules', String, self.modules, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.lidar_scan, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom, queue_size=1)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl, queue_size=1)
        rospy.Subscriber('/map', OccupancyGrid, self.map_update, queue_size=1)
        rospy.Subscriber('/inspection_camera/image_raw/compressed', CompressedImage,
                         self.camera, queue_size=1, buff_size=4*1024*1024)
        rospy.Subscriber('/thermal/image_color', Image, self.thermal, queue_size=1, buff_size=262144)
        # Calibrated 32x24 thermal-to-RGB homography is applied upstream by the
        # EXISTING stip_thermal_camera/thermal_rgb_overlay.py ROS node.
        rospy.Subscriber('/thermal/rgb_overlay', Image, self.registered_overlay, queue_size=1, buff_size=4*1024*1024)
        rospy.Subscriber('/ArmAngleUpdate', ArmJoint, self.arm_pose, queue_size=2)
        rospy.Subscriber('/fire_inspection/request_navigation_pause', Bool,
                         self.fire_warning, queue_size=1)
        rospy.Subscriber('/fire_inspection/confirmed', Bool, self.fire_confirmed, queue_size=1)
        rospy.Subscriber('/fire_inspection/state', String, lambda m: self.set_value('fire_state',m.data), queue_size=1)
        rospy.Subscriber('/fire_inspection/status_json',String,self.fire_status_update,queue_size=1)
        rospy.Subscriber('/fire_inspection/target_hot_c',Float32,lambda m:self.set_value('fire_target_hot_c',round(m.data,1)),queue_size=1)
        rospy.Subscriber('/fire_inspection/background_c',Float32,lambda m:self.set_value('fire_background_c',round(m.data,1)),queue_size=1)
        rospy.Subscriber('/thermal/healthy', Bool, lambda m: self.health('thermal', m.data), queue_size=1)
        rospy.Subscriber('/gas_sensor/healthy', Bool, lambda m: self.health('gas', m.data), queue_size=1)
        rospy.Subscriber('/gas_sensor/alarm_state', Bool, lambda m:self.set_value('gas_alarm_state',bool(m.data)), queue_size=1)
        rospy.Subscriber('/fire_detection/enabled', Bool, self.fire_enabled, queue_size=1)
        rospy.Subscriber('/gas_sensor/raw', Int32, lambda m: self.set_value('gas_raw', int(m.data)), queue_size=1)
        rospy.Subscriber('/gas_sensor/change_percent', Float32,
                         lambda m: self.set_value('gas_change_percent', round(m.data, 1)), queue_size=1)
        rospy.Subscriber('/inspection/geometry', Float64MultiArray, self.inspection_geometry, queue_size=1)
        rospy.Subscriber('/fire_detection/detections', Float64MultiArray, self.fire_detections, queue_size=1)
        rospy.Subscriber('/gas_sensor/calibration_status', String, lambda m:self.set_value('gas_calibration_status',m.data), queue_size=1)
        rospy.Subscriber('/gas_sensor/baseline', Int32, lambda m:self.set_value('gas_baseline',int(m.data)), queue_size=1)
        rospy.Subscriber('/gas_sensor/status', String, lambda m:self.set_value('gas_status',m.data), queue_size=1)
        threading.Thread(target=self.arm_hold_loop, daemon=True).start()
        threading.Thread(target=self.teleop_watchdog, daemon=True).start()
        threading.Thread(target=self.report_loop, daemon=True).start()

    def set_value(self, k, v):
        with self.lock:
            self.state[k] = v
            if k == 'gas_change_percent':
                self.state['gas_sample_mono'] = time.monotonic()

    def health(self, k, v):
        with self.lock: self.state['health'][k] = bool(v)

    def modules(self, msg):
        try:
            with self.lock: self.state['modules'] = json.loads(msg.data)
        except ValueError: pass

    def inspection_geometry(self, msg):
        geometry=parse_qr_geometry(msg.data)
        if geometry:
            geometry['updated_mono']=time.monotonic()
            with self.lock:self.state['inspection_geometry']=geometry

    def fire_detections(self, msg):
        geometry=parse_fire_detections(msg.data)
        if geometry:
            geometry['updated_mono']=time.monotonic()
            with self.lock:self.state['fire_detections']=geometry

    def fire_status_update(self,msg):
        try:
            value=json.loads(msg.data)
            if isinstance(value,dict):
                with self.lock:
                    self.state['fire_fusion']=value
        except (ValueError,TypeError):pass

    def save_fire_confirmation(self,metadata,rgb,thermal):
        try:
            # status_json is published just after the confirmation Boolean.
            # Capture camera bytes at the rising edge, but allow the associated
            # fusion telemetry one ROS callback cycle to arrive.
            time.sleep(.18)
            with self.lock:
                metadata['fusion']=dict(self.state.get('fire_fusion') or metadata['fusion'])
                metadata['hot_c']=self.state.get('fire_target_hot_c',metadata.get('hot_c'))
                metadata['background_c']=self.state.get('fire_background_c',metadata.get('background_c'))
                metadata['fusion_state']=self.state.get('fire_state',metadata.get('fusion_state'))
            identifier=save_event(self.fire_evidence_dir,metadata,rgb,thermal)
            emit('event',message='Fire confirmation captured once: '+identifier)
        except Exception as exc:
            emit('diagnostic',message='Fire evidence save error: '+str(exc)[:180])

    def record_inspection_result(self, result):
        with self.lock:
            results=self.state['inspection_results']
            results.append(result)
            del results[:-25]

    def lidar_scan(self, msg):
        now=time.monotonic()
        with self.lock: self.scan_time=now
        if now-self.last_laser_send<0.16:return
        self.last_laser_send=now
        # The scan frame is NOT the map frame. Transform once at the latest
        # available TF to avoid fake locations if map->odom is changing.
        try:
            trans, quat=self.tf_listener.lookupTransform('map',msg.header.frame_id,rospy.Time(0))
            yaw=math.atan2(2*(quat[3]*quat[2]+quat[0]*quat[1]),1-2*(quat[1]**2+quat[2]**2))
            ca,sa=math.cos(yaw),math.sin(yaw)
            skip=max(1,int(len(msg.ranges)/250))
            points=[]
            for i in range(0,len(msg.ranges),skip):
                dist=msg.ranges[i]
                if not math.isfinite(dist) or not msg.range_min<=dist<=min(msg.range_max,12.0):continue
                angle=msg.angle_min+i*msg.angle_increment
                lx,ly=dist*math.cos(angle),dist*math.sin(angle)
                points.append([round(trans[0]+ca*lx-sa*ly,3),round(trans[1]+sa*lx+ca*ly,3)])
            with self.lock:self.state['laser']={'points':points,'age_ms':0}
        except Exception as exc:
            if now-self.last_tf_warning>12:
                self.last_tf_warning=now
                emit('diagnostic', message='Laser overlay awaiting map TF: {}'.format(str(exc)[:110]))

    def odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y),1 - 2*(q.y*q.y + q.z*q.z))
        with self.lock:
            self.state['robot'] = {'x': round(msg.pose.pose.position.x, 3),
                                   'y': round(msg.pose.pose.position.y, 3), 'yaw': round(yaw, 3),
                                   'vx': round(msg.twist.twist.linear.x, 3),
                                   'omega': round(msg.twist.twist.angular.z, 3)}

    def amcl(self, msg):
        self.last_amcl_time = time.monotonic()
        cov = msg.pose.covariance
        pose=msg.pose.pose
        q=pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        with self.lock:
            self.state['amcl_map_pose']={'x':round(pose.position.x,3), 'y':round(pose.position.y,3),
                                         'yaw':round(yaw,4), 'source':'amcl'}
            if time.monotonic() > self.pose_command_time and self.pose_command_time:
                self.amcl_samples += 1
            good = self.amcl_samples >= 2 and 0 <= cov[0] <= 1.0 and 0 <= cov[7] <= 1.0 and 0 <= cov[35] <= 0.6
            self.state['localization'] = {'ready': bool(good), 'x_var': round(cov[0], 3),
                                           'y_var': round(cov[7], 3), 'yaw_var': round(cov[35], 3),
                                           'operator_confirmed':self.operator_confirmed}

    def map_update(self, msg):
        now=time.monotonic()
        if now-self.last_map<0.7:return
        self.last_map=now
        try:
            import cv2
            import numpy as np
            w,h=msg.info.width,msg.info.height
            if w*h>8000000 or not w or not h:return
            raw=np.asarray(msg.data,dtype=np.int16).reshape(h,w)
            # Unknown space becomes TRANSPARENT; dashboard's map grid shows
            # underneath. Keep a margin around known cells while SLAM grows.
            known=np.where(raw>=0)
            res=float(msg.info.resolution)
            if len(known[0])>0 and res>0:
                margin=max(12,round(2.0/res))
                x0=max(0,int(known[1].min())-margin)
                y0=max(0,int(known[0].min())-margin)
                x1=min(w,int(known[1].max())+margin+1)
                y1=min(h,int(known[0].max())+margin+1)
                # Do not zoom an almost-empty map into a giant noisy pixel.
                raw=raw[y0:y1,x0:x1]
            else:x0=y0=0
            hh,ww=raw.shape
            bgra=np.zeros((hh,ww,4),dtype=np.uint8)
            bgra[raw==0]=[230,230,230,245]
            bgra[(raw>0)&(raw<65)]=[155,155,155,210]
            bgra[raw>=65]=[24,29,37,255]
            # Orientation: ROS occupancy cells start in the lower left.
            bgra=cv2.flip(bgra,0)
            scale=min(1.,1300./max(ww,hh))
            if scale<1:
                bgra=cv2.resize(bgra,None,fx=scale,fy=scale,interpolation=cv2.INTER_AREA)
            ok,png=cv2.imencode('.png',bgra,[cv2.IMWRITE_PNG_COMPRESSION,2])
            if not ok:return
            q=msg.info.origin.orientation
            angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            ca,sa=math.cos(angle),math.sin(angle)
            ox=msg.info.origin.position.x+x0*res*ca-y0*res*sa
            oy=msg.info.origin.position.y+x0*res*sa+y0*res*ca
            meta={'width':ww,'height':hh,'resolution':res,'origin':[ox,oy],
                  'origin_yaw':angle,'frame':msg.header.frame_id or 'map',
                  'version':int(now*1000),'cropped':True}
            emit('map',image=base64.b64encode(png.tobytes()).decode('ascii'),meta=meta)
        except Exception as exc:
            emit('diagnostic',message='Map encoding: {}'.format(exc))

    def camera(self, msg):
        # Preserve original full-resolution JPEG for QR detection/enrollment.
        raw = bytes(msg.data)
        with self.lock:
            self.cached_jpeg = (raw, time.monotonic())
        now = time.monotonic()
        if now - self.last_cam < 0.065:
            return
        self.last_cam = now
        # The ROS topic is already JPEG compressed. Avoid JPEG decode/resize/
        # re-encode on every preview frame, which caused 2-3 fps on Jetson.
        if raw.startswith(b'\xff\xd8'):
            if self.camera_frame_path:
                write_frame(self.camera_frame_path,raw)
            else:
                emit('camera', jpeg=base64.b64encode(raw).decode('ascii'))

    def registered_overlay(self,msg):
        """Bridge calibrated BGR8 from the working thermal package; no second transform."""
        if not self.registered_frame_path or time.monotonic()-self.last_registered<0.13:
            return
        try:
            import numpy as np, cv2
            if msg.encoding.lower() not in ('bgr8','rgb8') or msg.width<32 or msg.height<24 or msg.step<msg.width*3:
                return
            b=np.frombuffer(bytes(msg.data),np.uint8)
            if len(b)<msg.step*msg.height:return
            img=b[:msg.step*msg.height].reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
            if msg.encoding.lower()=='rgb8':img=cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
            ok,encoded=cv2.imencode('.jpg',img,[cv2.IMWRITE_JPEG_QUALITY,72])
            if ok:
                write_frame(self.registered_frame_path,encoded.tobytes())
                self.last_registered=time.monotonic()
        except Exception as exc:
            if time.monotonic()-self.last_registered>10:
                self.last_registered=time.monotonic()
                emit('diagnostic',message='Calibrated overlay stream error: '+str(exc)[:120])

    def thermal(self,msg):
        # MLX90640 is ~8 complete FPS at 16-Hz refresh; cap preview at ~6 FPS.
        if time.monotonic()-self.last_thermal<.16:return
        self.last_thermal=time.monotonic()
        try:
            import numpy as np,cv2
            channels=3
            a=np.frombuffer(bytes(msg.data),dtype=np.uint8)
            raw=a.reshape(msg.height,msg.step)[:,:msg.width*channels].reshape(msg.height,msg.width,channels)
            if msg.encoding.lower()=='rgb8':raw=cv2.cvtColor(raw,cv2.COLOR_RGB2BGR)
            if msg.width>480:raw=cv2.resize(raw,(480,int(msg.height*480/msg.width)))
            ok,jpeg=cv2.imencode('.jpg',raw,[cv2.IMWRITE_JPEG_QUALITY,60])
            if ok:
                with self.lock:self.fire_thermal_snapshot=jpeg.tobytes()
                emit('thermal',jpeg=base64.b64encode(jpeg.tobytes()).decode('ascii'))
        except Exception:pass

    def arm_pose(self, msg):
        with self.lock:
            if msg.id==1:
                self.state['arm']['pan']=float(msg.angle)
                self.arm_hold.observe('pan',msg.angle)
            if msg.id==2:
                self.state['arm']['tilt']=float(msg.angle)
                self.arm_hold.observe('tilt',msg.angle)

    def sync_fire_hold(self):
        """A monitoring pause is transient; a confirmed ROS alarm is safety critical.

        Never resume a stopped mission automatically when a warning disappears.
        The working fire controller owns the confirmed-alarm clear decision.
        """
        with self.lock:
            monitored = bool(self.state.get('fire'))
            confirmed = bool(self.state.get('confirmed'))
            requested = bool(self.fire_pause_requested)
            active, reason = evaluate_fire_hold(monitored, requested, confirmed, self.state.get('fire_state','UNKNOWN'))
            was_active = self.fire_hold
            self.fire_hold = active
            self.state['fire_hold'] = active
            self.state['warning'] = bool(requested and monitored)
            self.state['fire_hold_reason'] = reason
        if active and not was_active:
            self.stop_motion()  # Cancels goals, drives zero, and pauses work.
            self.pause_ack.publish(Bool(data=True))
            emit('event', message='Fire inspection requested a motion hold: '+reason)
        elif was_active and not active:
            # Only release the interlock, never restart/cancelled goals automatically.
            self.pause_ack.publish(Bool(data=False))
            emit('event', message='Fire hold released after inspection status cleared; patrol remains stopped')

    def fire_enabled(self, msg):
        self.set_value('fire', bool(msg.data))
        self.sync_fire_hold()

    def fire_warning(self, msg):
        with self.lock:
            self.fire_pause_requested = bool(msg.data)
        self.sync_fire_hold()

    def fire_confirmed(self, msg):
        with self.lock:
            prior = bool(self.state.get('confirmed'))
            self.state['confirmed'] = bool(msg.data)
        if msg.data and not prior:
            with self.lock:
                camera=self.cached_jpeg
                thermal=self.fire_thermal_snapshot
                fusion=dict(self.state.get('fire_fusion') or {})
                state=self.state.get('fire_state','UNKNOWN')
                hot=self.state.get('fire_target_hot_c')
                background=self.state.get('fire_background_c')
            metadata={'source':'stip_fire_inspection','status':'CONFIRMED',
                      'fusion_state':state,'hot_c':hot,'background_c':background,
                      'fusion':fusion}
            threading.Thread(target=self.save_fire_confirmation,
                args=(metadata,camera[0] if camera and time.monotonic()-camera[1]<2 else None,thermal),daemon=True).start()
            self.record_inspection_result({'kind':'fire','status':'ALERT',
                'profile':'Fire confirmation','waypoint':'',
                'detail':'Fire inspection confirmed; navigation hold active',
                'timestamp':time.strftime('%H:%M:%S')})
        self.sync_fire_hold()

    def stop_motion(self):
        # STOP wins over an in-flight teleop HTTP/RPC request.
        with self.motion_lock:
            self.teleop_armed=False
            self.teleop_session+=1
            self.last_motion_active=False
            self.pause.set()
            self.goal_cancel.set()
            if self.state.get('nav')=='MOVING':
                self.state['nav']='STOPPED'
            self.arm_hold.stop()
            try:self.client.cancel_all_goals()
            except Exception: pass
            for _ in range(3): self.cmd_vel.publish(Twist())

    def teleop_watchdog(self):
        rate=rospy.Rate(12)
        while not rospy.is_shutdown():
            # The teleop can stay ARMED while idle. Only motion needs a
            # heartbeat. Disarming after 350ms idle caused false rejections.
            # Allow bounded browser/network jitter, but never allow a stale
            # command to drive indefinitely. This stops MOTION, not the arm state.
            with self.motion_lock:
                expired=(self.teleop_armed and self.last_motion_active and
                         time.monotonic()-self.last_teleop>0.60)
                if expired:
                    self.last_motion_active=False
                    self.cmd_vel.publish(Twist())
            if expired:emit('event', message='Teleop heartbeat gap >600 ms: motor zero (control remains armed)')
            rate.sleep()

    def report_loop(self):
        rate=rospy.Rate(5)
        while not rospy.is_shutdown():
            try:
                # A rospy process may remain alive while roscore is down.
                # Report the actual master RPC status; never fabricate READY.
                from rosgraph import Master
                Master('/stip_dashboard_adapter').getPid()
                online=True
            except Exception:
                online=False
            if online:
                try:
                    trans,quat=self.tf_listener.lookupTransform('map','base_footprint',rospy.Time(0))
                    yaw=math.atan2(2*(quat[3]*quat[2]+quat[0]*quat[1]),1-2*(quat[1]**2+quat[2]**2))
                    with self.lock:self.state['robot_map']={'x':round(trans[0],3),'y':round(trans[1],3),'yaw':round(yaw,4),'source':'tf'}
                except Exception:
                    try:
                        trans,quat=self.tf_listener.lookupTransform('map','base_link',rospy.Time(0))
                        yaw=math.atan2(2*(quat[3]*quat[2]+quat[0]*quat[1]),1-2*(quat[1]**2+quat[2]**2))
                        with self.lock:self.state['robot_map']={'x':round(trans[0],3),'y':round(trans[1],3),'yaw':round(yaw,4),'source':'tf'}
                    except Exception:
                        with self.lock:
                            amcl_pose=self.state.get('amcl_map_pose')
                            if amcl_pose and time.monotonic()-self.last_amcl_time<2.5:
                                self.state['robot_map']={k:amcl_pose[k] for k in ('x','y','yaw','source')}
                            else:
                                self.state.pop('robot_map',None)
            if not online:
                # Clear stale coordinates after ROS master disconnect. Confirmation
                # must be repeated after the TF/navigation connection recovers.
                with self.lock:
                    self.state.pop('robot_map',None)
                    self.operator_confirmed=False
            if online and time.monotonic()-self.last_arm_publishers_check>1.0:
                self.last_arm_publishers_check=time.monotonic()
                try:
                    import rosgraph
                    pubs,_,_=rosgraph.Master(rospy.get_name()).getSystemState()
                    arm_publishers=next((sorted(nodes) for topic,nodes in pubs if topic=='/TargetAngle'),[])
                    with self.lock:self.state['arm_publishers']=arm_publishers
                except Exception:
                    with self.lock:self.state.pop('arm_publishers',None)
            # Query ROS publisher connections outside the shared state lock.
            # Holding it during ROS transport checks could block a fast operator
            # localization-confirmation request.
            initialpose_subscribed=self.pose_pub.get_num_connections()>0 if online else False
            with self.lock:
                self.state['localization']['initialpose_subscribers']=initialpose_subscribed
                self.state['teleop']={'armed':self.teleop_armed,'moving':self.last_motion_active,
                                      'session':self.teleop_session}
                self.state['connected']=online
                for overlay in ('inspection_geometry','fire_detections'):
                    data=self.state.get(overlay)
                    if data and time.monotonic()-data.get('updated_mono',0)>2.0:
                        self.state.pop(overlay,None)
                self.state['health']['lidar']=bool(online and self.scan_time and time.monotonic()-self.scan_time < 3.0)
                if time.monotonic()-self.last_amcl_time>3.0:
                    self.state['localization']['ready']=False
                # Every report must revalidate pose source freshness. Never publish a stale map pose.
                if not self.state.get('robot_map') or not usable_map_pose(self.state['robot_map']):
                    self.state['robot_map']=None
                snapshot=json.loads(json.dumps(self.state))
            emit('state', data=snapshot)
            rate.sleep()

    def arm_publisher(self):
        """Get a live camera-arm publisher while holding arm_pub_lock.

        Caller must hold arm_pub_lock until it has published. ROS Publisher's
        impl becomes None after unregister(); checking it concurrently is
        the cause of NoneType.get_num_connections on the Jetson.
        """
        with self.arm_pub_lock:
            if self.arm_pub is None or getattr(self.arm_pub, 'impl', True) is None:
                self.arm_pub=rospy.Publisher('/TargetAngle', ArmJoint, queue_size=2)
            pub=self.arm_pub
            deadline=time.monotonic()+0.8
            while not rospy.is_shutdown():
                try:
                    subscribers=pub.get_num_connections()
                except AttributeError:
                    # Already unregistered by rospy; replace instead of
                    # surfacing an internal NoneType exception to the operator.
                    self.arm_pub=rospy.Publisher('/TargetAngle', ArmJoint, queue_size=2)
                    pub=self.arm_pub
                    subscribers=pub.get_num_connections()
                if subscribers>0:
                    return pub
                if time.monotonic()>=deadline:
                    break
                time.sleep(.04)
            raise ValueError('No /TargetAngle subscriber; check robot arm driver')

    def release_arm_publisher(self):
        """Release only when no arm command is reading/publishing via rospy."""
        with self.arm_pub_lock:
            pub=self.arm_pub
            self.arm_pub=None
            if pub is not None:
                pub.unregister()

    def arm_hold_loop(self):
        # Driver commands are generated at a bounded rate, not queued by browser.
        while not rospy.is_shutdown():
            try:
                if self.fire_hold or self.state['mission'].get('state') in ('RUNNING','PAUSED'):
                    self.arm_hold.stop()
                else:
                    step=self.arm_hold.tick()
                    if step:
                        axis,angle=step
                        self.send_arm(axis,angle)
                # Idle dashboard must not permanently block the ROS fire
                # controller's exclusive Publisher arbitration.
                if self.arm_pub is not None and self.arm_hold.key is None and \
                        time.monotonic()-self.last_arm_command>2.0:
                    self.release_arm_publisher()
            except Exception as exc:
                self.arm_hold.stop()
                emit('diagnostic',message='Camera arm jog stopped: '+str(exc)[:120])
            time.sleep(.04)

    def arm_home(self, reason='idle'):
        """Return to QR package's configured home without stealing fire auto-aim.

        This is a bounded command request, not proof that physical joints have
        reached home. If a fire hold is active, fire inspection owns the arm.
        """
        if self.fire_hold:
            emit('event', message='Arm home deferred: fire inspection owns camera arm')
            return False
        self.arm_hold.stop()
        pan=float(rospy.get_param('/stip_qr_inspection/pan_home',90.0))
        tilt=float(rospy.get_param('/stip_qr_inspection/tilt_home',0.0))
        # Use a gentle, bounded return and let the last target settle.
        self.send_arm('pan',pan,run_time=650)
        time.sleep(.16)
        self.send_arm('tilt',tilt,run_time=650)
        time.sleep(.7)
        emit('event',message='Camera arm home requested ({})'.format(reason))
        return True

    def send_arm(self, axis, angle, run_time=130):
        if self.fire_hold: raise ValueError('Arm unavailable during fire hold')
        bounds=(0,180) if axis=='pan' else (0,90)
        if not bounds[0]<=angle<=bounds[1]:raise ValueError('Angle outside servo limits')
        msg=ArmJoint();msg.id=1 if axis=='pan' else 2;msg.angle=angle;msg.run_time=max(100,min(2000,int(run_time)))
        # Keep publisher validation and publish under the SAME lock as
        # unregister. Otherwise idle release can invalidate rospy's impl
        # between get_num_connections() and publish().
        with self.arm_pub_lock:
            publisher=self.arm_publisher()
            publisher.publish(msg)
            self.last_arm_command=time.monotonic()
        with self.lock:self.state['arm'][axis]=angle
        return {'axis':axis, 'angle':angle}

    def initialpose(self, payload):
        if self.state.get('mission',{}).get('state') in ('RUNNING','PAUSED'): raise ValueError('Stop patrol before changing localization')
        if self.pose_pub.get_num_connections()<=0:
            raise ValueError('AMCL has not subscribed to /initialpose yet; wait for navigation startup')
        x,y,yaw=float(payload['x']),float(payload['y']),float(payload['yaw'])
        msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.header.stamp=rospy.Time.now()
        msg.pose.pose.position.x=x;msg.pose.pose.position.y=y
        msg.pose.pose.orientation.z=math.sin(yaw/2);msg.pose.pose.orientation.w=math.cos(yaw/2)
        msg.pose.covariance[0]=0.25;msg.pose.covariance[7]=0.25;msg.pose.covariance[35]=0.0685
        with self.lock:
            self.pose_command_time=time.monotonic()+0.05
            self.amcl_samples=0
            self.operator_confirmed=False
            self.state['localization']={'ready':False, 'message':'Waiting for fresh AMCL estimates'}
        for _ in range(3): self.pose_pub.publish(msg);rospy.sleep(.08)
        return {'accepted':True,'localized':False}

    def goal(self, point):
        # Mission Editor navigation and manual teleop must not both command /cmd_vel.
        if self.teleop_armed:raise ValueError('Release Teleop before sending a navigation goal')
        if not self.state.get('robot_map'):raise ValueError('Map-frame robot pose unavailable. Check map-to-base TF or /amcl_pose')
        if self.fire_hold: raise ValueError('Navigation blocked by fire hold')
        if not self.client.wait_for_server(rospy.Duration(1.0)):raise ValueError('move_base unavailable')
        x,y,yaw=(float(point[k]) for k in ('x','y','yaw'))
        goal=MoveBaseGoal();goal.target_pose.header.frame_id='map';goal.target_pose.header.stamp=rospy.Time.now()
        goal.target_pose.pose.position.x=x;goal.target_pose.pose.position.y=y
        goal.target_pose.pose.orientation.z=math.sin(yaw/2)
        goal.target_pose.pose.orientation.w=math.cos(yaw/2)
        # Lock the final send and owner transition against Teleop ARM and STOP.
        # The server existence check above is intentionally outside the lock.
        with self.motion_lock:
            if self.teleop_armed:raise ValueError('Release Teleop before sending a navigation goal')
            if self.fire_hold:raise ValueError('Navigation blocked by fire hold')
            self.goal_cancel.clear()
            with self.lock:
                self.state['arrived_id']=None
                self.state['nav']='MOVING'
            self.client.send_goal(goal)
        emit('event', message='Navigation goal sent: {} (x {:.2f}, y {:.2f}, yaw {:.1f}°)'.format(point.get('name',point.get('id','Waypoint')),x,y,math.degrees(yaw)))
        def check_arrival(pointid=point.get('id'), started=time.monotonic()):
            from actionlib_msgs.msg import GoalStatus
            while time.monotonic()-started<245 and not rospy.is_shutdown():
                result=self.client.get_state()
                if result==GoalStatus.SUCCEEDED and not self.goal_cancel.is_set():
                    with self.lock:self.state['nav']='ARRIVED';self.state['arrived_id']=pointid
                    emit('event',message='Arrived at waypoint: {}'.format(point.get('name',pointid or 'Waypoint')))
                    break
                if result in (GoalStatus.ABORTED,GoalStatus.REJECTED,GoalStatus.PREEMPTED,GoalStatus.LOST):
                    if self.goal_cancel.is_set():break  # Operator cancellation is not a navigation failure.
                    with self.lock:self.state['nav']='FAILED'
                    emit('event',message='Navigation aborted/failed: {} (action state {})'.format(point.get('name',pointid or 'Waypoint'),result))
                    break
                time.sleep(.1)
        threading.Thread(target=check_arrival,daemon=True).start()
        return {'accepted':True}

    def mission_runner(self, mission):
        from actionlib_msgs.msg import GoalStatus
        self.set_value('mission',{'state':'RUNNING','id':mission['id'],'index':0,'task':''})
        emit('event',message='Patrol started: {}'.format(mission.get('name',mission['id'])))
        try:
            while not self.patrol_halt.is_set() and not rospy.is_shutdown():
                for index,point in enumerate(mission['waypoints']):
                    if self.patrol_halt.is_set():break
                    with self.lock:self.state['mission']['index']=index+1
                    while self.pause.is_set() and not self.patrol_halt.is_set():time.sleep(.1)
                    if self.patrol_halt.is_set():break
                    self.goal(point)
                    deadline=time.monotonic()+240
                    while not self.patrol_halt.is_set():
                        if self.fire_hold or self.goal_cancel.is_set():raise RuntimeError('Navigation interrupted; operator intervention required')
                        if self.pause.is_set():
                            self.client.cancel_all_goals()
                            with self.lock:self.state['mission']['state']='PAUSED'
                            while self.pause.is_set() and not self.patrol_halt.is_set():time.sleep(.1)
                            if self.patrol_halt.is_set():break
                            self.goal(point)
                            with self.lock:self.state['mission']['state']='RUNNING'
                        result=self.client.get_state()
                        if result==GoalStatus.SUCCEEDED:break
                        if result in (GoalStatus.ABORTED,GoalStatus.REJECTED,GoalStatus.LOST):raise RuntimeError('move_base goal failed: {}'.format(result))
                        if time.monotonic()>deadline:raise RuntimeError('Navigation timeout')
                        time.sleep(.1)
                    if self.patrol_halt.is_set():break
                    self.cmd_vel.publish(Twist())
                    for task in point.get('tasks',[]):
                        if self.patrol_halt.is_set() or self.fire_hold:break
                        with self.lock:self.state['mission']['task']=task['type']
                        if task['type']=='pause':
                            left=float(task['seconds'])
                            while left>0 and not self.patrol_halt.is_set():
                                if self.pause.is_set():time.sleep(.1);continue
                                time.sleep(min(.1,left));left-=.1
                        elif task['type']=='qr' and self.qr_enabled:
                            # Original /inspection/run service performs scan,
                            # alignment and evidence persistence. This adapter
                            # only positions the arm and owns the safety cleanup.
                            response = None
                            try:
                                self.arm_home('before QR inspection')
                                if self.patrol_halt.is_set() or self.fire_hold:
                                    raise RuntimeError('QR inspection interrupted before start')
                                self.send_arm('pan',float(task.get('pan',90)))
                                self.send_arm('tilt',float(task.get('tilt',0)))
                                time.sleep(.7)
                                rospy.wait_for_service('/inspection/run',timeout=3)
                                response=rospy.ServiceProxy('/inspection/run',RunInspection)(task['profile'])
                                self.record_inspection_result({'kind':'qr','profile':task['profile'],
                                    'waypoint':point.get('name',point.get('id','')),
                                    'status':'PASS' if response.success else 'FAIL',
                                    'detail':response.message[:240], 'qr_value':response.qr_value[:160],
                                    'pan':round(float(response.pan),1),'tilt':round(float(response.tilt),1),
                                    'timestamp':time.strftime('%H:%M:%S')})
                                emit('event',message='QR {}: {}'.format(task['profile'],response.message))
                                if not response.success:
                                    raise RuntimeError('QR task failed: {}'.format(response.message))
                            except Exception as exc:
                                if response is None:
                                    self.record_inspection_result({'kind':'qr','profile':task['profile'],
                                        'waypoint':point.get('name',point.get('id','')),
                                        'status':'ERROR','detail':str(exc)[:240],
                                        'timestamp':time.strftime('%H:%M:%S')})
                                raise
                            finally:
                                # Do not fight active fire-inspection auto-aim.
                                if not self.fire_hold:
                                    try:self.arm_home('after QR inspection')
                                    except Exception as exc:emit('event',message='Arm home failed: '+str(exc))
                    with self.lock:self.state['mission']['task']=''
                if not mission.get('loop') or self.patrol_halt.is_set():break
            with self.lock:self.state['mission']['state']='STOPPED' if self.patrol_halt.is_set() else 'COMPLETED'
            emit('event',message='Patrol {}: {}'.format('stopped' if self.patrol_halt.is_set() else 'completed',mission.get('name',mission['id'])))
        except Exception as exc:
            self.stop_motion()
            with self.lock:self.state['mission']['state']='ERROR';self.state['mission']['error']=str(exc)
            emit('event',message='Mission ERROR: {}'.format(exc))
        finally:
            self.cmd_vel.publish(Twist())

    def qr_probe(self):
        """Read-only detection using the same ZBar-first approach as QR inspection."""
        with self.lock: item = self.cached_jpeg
        if not item or time.monotonic()-item[1]>2:
            return {'detected':False, 'detail':'Waiting for a recent RGB camera frame'}
        import cv2
        import numpy as np
        frame=cv2.imdecode(np.frombuffer(item[0],np.uint8),cv2.IMREAD_COLOR)
        value, points=decode_qr(frame)
        return {'detected':bool(value and points is not None),
                'value':value[:120] if value else '',
                'detail':'QR detected' if value else 'No readable QR in latest camera frame'}

    def qr_prepare(self, payload):
        """Retry a handful of FRESH frames, then freeze the frame actually decoded.

        Camera is allowed to continue publishing while we check: the last frame
        is often blurred from a recent pan/tilt command. Never attach an ROI to
        an image different from the detected frame.
        """
        if self.fire_hold: raise ValueError('Fire hold prevents QR enrollment')
        if self.last_motion_active: raise ValueError('Stop the robot before QR enrollment')
        import cv2
        import numpy as np
        deadline=time.monotonic()+1.8
        checked=0
        previous_stamp=None
        had_fresh=False
        while time.monotonic()<deadline:
            with self.lock: item=self.cached_jpeg
            if not item or time.monotonic()-item[1]>2:
                time.sleep(.06)
                continue
            had_fresh=True
            raw,stamp=item
            if stamp==previous_stamp:
                time.sleep(.05)
                continue
            previous_stamp=stamp
            frame=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
            if frame is None:
                continue
            checked+=1
            value,points=decode_qr(frame)
            if value and points is not None:
                fid=str(time.time_ns())
                with self.lock:
                    self.frame_capture={fid:(raw,time.monotonic(),value,np.asarray(points,dtype=float).reshape(-1,2).tolist())}
                return {'detected':True,'frame_id':fid,
                        'jpeg':'data:image/jpeg;base64,'+base64.b64encode(raw).decode('ascii'),
                        'qr_value':value[:120],
                        'qr_points': np.asarray(points,dtype=float).reshape(-1,2).tolist()}
        if not had_fresh:
            return {'detected':False,'detail':'No recent RGB camera frames. Check /inspection_camera/image_raw/compressed.'}
        return {'detected':False,'detail':'QR not decoded from {} fresh frame(s). Hold the camera steady and move closer.'.format(checked)}

    def qr_capture(self,payload):
        # QR profile creation is an operator-driven editing action. It does not
        # require executing a navigation goal to a waypoint already occupied.
        # Patrol navigation still independently verifies goal arrival.
        if self.fire_hold:raise ValueError('Fire hold prevents QR capture')
        if self.last_motion_active:raise ValueError('Stop driving before capturing QR')
        with self.lock: value=self.cached_jpeg
        if not value or time.monotonic()-value[1]>2:raise ValueError('No recent RGB image')
        raw=value[0]
        fid=str(int(time.time()*1000))
        self.frame_capture={fid: (raw, time.monotonic())}
        return {'frame_id':fid,'jpeg':'data:image/jpeg;base64,'+base64.b64encode(raw).decode('ascii')}

    def qr_enroll(self, payload):
        if self.fire_hold:raise ValueError('Fire hold prevents QR enrollment')
        name=payload['name']
        if not name.isascii() or not __import__('re').fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}',name):
            raise ValueError('Invalid QR enrollment name')
        fid=payload['frame_id']
        with self.lock:
            item=self.frame_capture.get(fid)
        if not item or time.monotonic()-item[1]>900:
            raise ValueError('QR capture no longer available. Choose Back to Live and detect the QR again.')
        import cv2
        import numpy as np
        raw=item[0]
        frame=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
        if frame is None:raise ValueError('Could not decode saved QR frame')
        if len(item)>=4:
            qr_text=item[2]
            qr_points=np.asarray(item[3],dtype=np.float32).reshape(-1,2)
        else:
            qr_text,qr_points=decode_qr(frame)
        if not qr_text or qr_points is None:raise ValueError('No QR code in this frame')
        h,w=frame.shape[:2]
        box=payload['roi']
        x=int(float(box['x'])*w);y=int(float(box['y'])*h)
        rw=int(float(box['w'])*w);rh=int(float(box['h'])*h)
        if not(0<=x<w and 0<=y<h and rw>=5 and rh>=5 and x+rw<=w and y+rh<=h):
            raise ValueError('Draw valid indicator ROI inside captured frame')
        pan=self.state['arm']['pan'];tilt=self.state['arm']['tilt']
        # Compatible with original QR inspection profile schema.
        corners=np.asarray(qr_points,dtype=float).reshape(-1,2).tolist()
        indicator=[[x,y],[x+rw,y],[x+rw,y+rh],[x,y+rh]]
        path=Path.home()/'Documents'/'stip_inspection_data'/'profiles'/name
        path.mkdir(parents=True,exist_ok=True)
        if (path/'profile.json').exists():
            # Enrollment only commits when the operator clicks Save Enrollment.
            # Existing profiles are backed up automatically, no toggle required.
            # Note: profiles are global: other missions with this name read the update.
            import shutil,uuid
            backup=path.parent.parent/'profile_backups'/name/(time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
            backup.mkdir(parents=True,exist_ok=False)
            for filename in ('profile.json','reference.jpg','reference_annotated.jpg'):
                source=path/filename
                if source.is_file() and not source.is_symlink():
                    shutil.copy2(str(source),str(backup/filename))
        ref_tmp=path/'reference.jpg.tmp'
        ref_tmp.write_bytes(raw)
        os.replace(str(ref_tmp),str(path/'reference.jpg'))
        annotated=frame.copy()
        cv2.polylines(annotated,[np.asarray(corners,dtype=np.int32).reshape(-1,1,2)],True,(0,255,0),3)
        cv2.rectangle(annotated,(x,y),(x+rw,y+rh),(255,255,0),3)
        annot_tmp=path/'reference_annotated.tmp.jpg'
        if not cv2.imwrite(str(annot_tmp),annotated):raise ValueError('Could not save annotated QR image')
        os.replace(str(annot_tmp),str(path/'reference_annotated.jpg'))
        profile={'version':2,'name':name,'created_at':__import__('datetime').datetime.now().isoformat(),
                 'camera':{'width':w,'height':h},'camera_reference':{'pan':pan,'tilt':tilt},
                 'qr':{'value':qr_text,'corners':corners},
                 'indicator':{'bbox':{'x':x,'y':y,'w':rw,'h':rh},'corners':indicator},
                 'alignment':{'target':'indicator_center','tolerance_px':20.,'margin_px':30.},
                 'files':{'reference':str(path/'reference.jpg'),
                          'reference_annotated':str(path/'reference_annotated.jpg')}}
        temp=path/'profile.json.tmp'
        temp.write_text(json.dumps(profile,indent=2));os.replace(str(temp),str(path/'profile.json'))
        try:self.arm_home('after QR enrollment')
        except Exception as exc: emit('event',message='QR profile saved; camera arm home failed: '+str(exc))
        return {'profile':name,'qr_value':qr_text,'pan':pan,'tilt':tilt,'path':str(path/'profile.json')}

    def handle(self, name, payload):
        if name=='ping':return {'ros':True}
        if name=='stop':self.stop_motion();self.patrol_halt.set();return {'stopped':True}
        if name=='gas_alarm':
            enabled=bool(payload.get('enabled'))
            self.gas_alarm_pub.publish(Bool(data=enabled))
            return {'sent':True,'requested':enabled}  # Hardware ACK is /gas_sensor/alarm_state.
        if name=='patrol_takeover':
            if self.fire_hold or self.state.get('confirmed') or self.state.get('warning'):
                raise ValueError('Fire hold/warning active; manual motion is blocked')
            # Stop existing move_base execution permanently. Restarting Patrol
            # after manual driving begins the mission from its first waypoint.
            self.stop_motion()
            self.patrol_halt.set()
            runner=self.patrol_thread
            if runner and runner.is_alive():
                runner.join(timeout=2.5)
                if runner.is_alive():
                    raise ValueError('Patrol task still stopping; teleop was NOT armed')
            if self.fire_hold or self.state.get('confirmed') or self.state.get('warning'):
                raise ValueError('Fire condition appeared; teleop was NOT armed')
            with self.lock:
                self.state['mission']['state']='STOPPED'
                self.state['mission']['task']=''
                self.state['nav']='MANUAL'
            with self.motion_lock:
                self.teleop_session+=1
                self.teleop_armed=True
                self.teleop_seq=-1
                self.last_teleop=time.monotonic()
                self.last_motion_active=False
                self.cmd_vel.publish(Twist())
                session=self.teleop_session
            emit('event',message='Manual takeover: Patrol stopped, goal cancelled; teleop armed')
            return {'armed':True,'session':session,'patrol_stopped':True,'restart_from_beginning':True}
        if name=='teleop_arm':
            enabled=bool(payload.get('enabled'))
            with self.motion_lock:
                if enabled and (self.fire_hold or self.state['mission'].get('state') in ('RUNNING','PAUSED')):
                    raise ValueError('Motion blocked by an active mission or fire hold')
                if enabled and self.state.get('nav')=='MOVING':
                    raise ValueError('Cancel the active navigation goal before enabling Teleop')
                self.teleop_session+=1
                self.teleop_armed=enabled
                self.teleop_seq=-1
                self.last_teleop=time.monotonic()
                self.last_motion_active=False
                self.cmd_vel.publish(Twist())
                session=self.teleop_session
            emit('event',message='Manual Teleop {} (session {})'.format('ARMED' if enabled else 'DISARMED',session))
            return {'armed':enabled,'session':session}
        if name=='teleop':
            seq=int(payload.get('seq',0))
            session=payload.get('session')
            linear=max(-0.4,min(0.4,float(payload.get('linear',0))))
            angular=max(-1.5,min(1.5,float(payload.get('angular',0))))
            with self.motion_lock:
                # Packets from the previous arm/disarm epoch must never move
                # the robot after a new UI session starts.
                if session is None or int(session)!=self.teleop_session:
                    return {'sent':False,'ignored':'expired teleop session'}
                if seq<=self.teleop_seq:
                    return {'sent':False,'ignored':'stale teleop command'}
                self.teleop_seq=seq
                if not self.teleop_armed or self.fire_hold or self.state['mission'].get('state') in ('RUNNING','PAUSED'):
                    self.cmd_vel.publish(Twist())
                    self.last_motion_active=False
                    if abs(linear)<1e-6 and abs(angular)<1e-6:
                        return {'sent':False,'ignored':'already stopped/disarmed'}
                    raise ValueError('Teleop not armed or motion blocked: check patrol/fire state')
                msg=Twist();msg.linear.x=linear;msg.angular.z=angular
                self.cmd_vel.publish(msg)
                self.last_teleop=time.monotonic()
                self.last_motion_active=(abs(linear)>1e-6 or abs(angular)>1e-6)
                return {'sent':True,'session':session}
        if name=='localization_confirm':
            with self.lock:
                pose=self.state.get('robot_map')
                if not usable_map_pose(pose):
                    raise ValueError('No live map-frame pose; wait for map to base TF or AMCL')
                self.operator_confirmed=True
                self.state['localization']['operator_confirmed']=True
            return {'confirmed':True,'pose':pose}
        if name=='initialpose':return self.initialpose(payload)
        if name=='goal':return self.goal(payload)
        if name=='arm_hold':
            op=payload.get('op')
            if op=='stop':return {'accepted':self.arm_hold.stop(payload['seq'])}
            if self.fire_hold or self.state['mission'].get('state') in ('RUNNING','PAUSED'):
                self.arm_hold.stop()
                raise ValueError('Manual camera arm blocked during patrol or fire hold')
            if op!='pulse':raise ValueError('Invalid arm hold operation')
            self.arm_publisher()
            return {'accepted':self.arm_hold.pulse(payload['key'],payload['seq'])}
        if name=='arm':
            self.arm_hold.stop()
            return self.send_arm(payload['axis'],float(payload['angle']))
        if name=='qr_capture':return self.qr_capture(payload)
        if name=='qr_prepare':return self.qr_prepare(payload)
        if name=='qr_probe':return self.qr_probe()
        if name=='stage_restart':
            stage=payload['stage']
            if stage=='bringup':service='/stip/startup/retry_bringup'
            elif stage in ('ui','camera','qr','fire','gas','thermal'):
                service='/stip/modules/restart_'+stage
            else:raise ValueError('Not a managed stage')
            rospy.wait_for_service(service,timeout=3)
            reply=rospy.ServiceProxy(service,Trigger)()
            if not reply.success:raise ValueError(reply.message)
            return {'accepted':True,'message':reply.message}
        if name=='qr_test':
            profile=str(payload.get('profile',''))
            if not __import__('re').fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}',profile):
                raise ValueError('Invalid QR profile')
            if self.fire_hold or self.state['mission'].get('state') in ('RUNNING','PAUSED'):
                raise ValueError('Stop Patrol/fire hold before testing QR')
            if self.last_motion_active or self.teleop_armed:
                raise ValueError('Disarm teleop and stop the robot before testing QR')
            # No navigation goal. Run the existing /inspection/run service at
            # the robot's current location after aiming at enrolled angles.
            from qr_profiles import get_profile
            item=get_profile(Path.home()/'Documents'/'stip_inspection_data'/'profiles',profile)
            angles=item.get('camera_reference') or {}
            phase='returning arm home'
            try:
                self.arm_home('before QR test')
                phase='aiming at enrolled camera angles'
                for axis in ('pan','tilt'):
                    if axis in angles:self.send_arm(axis,float(angles[axis]),run_time=650)
                time.sleep(.7)
                # The ROS inspection package owns servo motion during search
                # and tracking. Release this dashboard publisher BEFORE its
                # service runs (do not compete for /TargetAngle ownership).
                self.release_arm_publisher()
                phase='waiting for /inspection/run service'
                rospy.wait_for_service('/inspection/run',timeout=3)
                phase='running the ROS inspection service'
                result=rospy.ServiceProxy('/inspection/run',RunInspection)(profile)
                self.record_inspection_result({'kind':'qr','profile':profile,'status':'PASS' if result.success else 'FAIL',
                   'detail':result.message[:240], 'qr_value':result.qr_value[:160],
                   'timestamp':time.strftime('%H:%M:%S')})
                return {'success':bool(result.success),'message':result.message,'qr_value':result.qr_value,
                        'annotated_path':result.annotated_path,'indicator_crop_path':result.indicator_crop_path}
            except Exception as exc:
                raise RuntimeError('QR test failed while {}: {}'.format(phase,exc))
            finally:
                if not self.fire_hold:
                    try:self.arm_home('after QR test')
                    except Exception as exc:
                        emit('diagnostic',message='QR test return-home failed: '+str(exc)[:180])
        if name=='qr_enroll':return self.qr_enroll(payload)
        if name=='gas_calibrate':
            if not payload.get('clean_air_confirmed'):
                raise ValueError('Confirm stabilized MQ-6 in clean air before calibration')
            if not self.state['health'].get('gas'):
                raise ValueError('MQ-6 not healthy. Check POP32i and gas bridge first')
            if self.state.get('gas_calibration_status')=='COLLECTING':
                raise ValueError('MQ-6 calibration is already collecting samples')
            rospy.wait_for_service('/gas_sensor/calibrate',timeout=3)
            response=rospy.ServiceProxy('/gas_sensor/calibrate',Trigger)()
            if not response.success:raise ValueError(response.message)
            return {'accepted':True,'message':response.message}
        if name=='fire_enable':
            enabled=bool(payload.get('enabled'))
            rospy.wait_for_service('/fire_detection/set_enabled',timeout=2)
            result=rospy.ServiceProxy('/fire_detection/set_enabled',SetBool)(enabled)
            if not result.success:raise ValueError(result.message)
            self.set_value('fire',enabled)
            self.sync_fire_hold()
            return {'enabled':enabled, 'fire_hold':self.fire_hold}
        if name=='fire_tracking':
            enabled=bool(payload.get('enabled'))
            if enabled and self.state['mission'].get('state')=='RUNNING' and not self.pause.is_set():
                raise ValueError('Pause robot motion before enabling fire auto-aim')
            if enabled:
                self.arm_hold.stop()
                self.release_arm_publisher()
            rospy.wait_for_service('/fire_inspection/set_auto_aim',timeout=3)
            r=rospy.ServiceProxy('/fire_inspection/set_auto_aim',SetBool)(enabled)
            if not r.success:raise ValueError(r.message)
            return {'enabled':enabled,'message':r.message}
        if name=='fire_clear':
            rospy.wait_for_service('/fire_inspection/acknowledge',timeout=3)
            response=rospy.ServiceProxy('/fire_inspection/acknowledge',Trigger)()
            if not response.success:raise ValueError(response.message)
            rospy.sleep(.35)
            if self.state.get('confirmed') or self.state.get('warning'):
                raise ValueError('Fire still asserted: hold retained; inspect scene and detector state')
            # Status must be deasserted by the ROS fire-inspection controller.
            self.sync_fire_hold()
            if self.fire_hold: raise ValueError('Fire controller still requests hold; check /fire_inspection/state')
            return {'cleared':True,'message':response.message}
        if name=='patrol_start':
            if self.teleop_armed:raise ValueError('Release manual teleop before starting patrol')
            if self.patrol_thread and self.patrol_thread.is_alive():raise ValueError('Patrol already active')
            if self.fire_hold:raise ValueError('Fire hold active. Clear the fire warning/confirmation before starting patrol.')
            # Continue on the browser is a local UI decision. At patrol START,
            # require that the operator confirmed that same session and that a
            # usable map-frame pose still exists. Do not gate the editor on AMCL
            # covariance or a separate localization_confirm RPC.
            if not (bool(payload.get('operator_confirmed')) or self.operator_confirmed):
                raise ValueError('Confirm the robot position before starting patrol')
            if not usable_map_pose(self.state.get('robot_map')):
                raise ValueError('Live map-frame pose required before robot motion')
            mission=payload['mission']
            if not mission.get('waypoints'):raise ValueError('No waypoints')
            self.mission=mission
            self.patrol_halt.clear();self.pause.clear()
            self.patrol_thread=threading.Thread(target=self.mission_runner,args=(mission,),daemon=True)
            self.patrol_thread.start();return {'accepted':True}
        if name=='patrol_pause':
            if self.teleop_armed or not (self.patrol_thread and self.patrol_thread.is_alive()):
                raise ValueError('No active Patrol to pause')
            self.pause.set();self.client.cancel_all_goals()
            emit('event',message='Patrol paused; navigation goal cancelled')
            return {'paused':True}
        if name=='patrol_resume':
            if self.teleop_armed or not (self.patrol_thread and self.patrol_thread.is_alive()) or not self.pause.is_set():
                raise ValueError('No paused Patrol to resume; use Start after manual control')
            if self.fire_hold:raise ValueError('Fire hold active. Clear the fire warning/confirmation before resuming patrol.')
            self.pause.clear()
            emit('event',message='Patrol resumed')
            return {'resumed':True}
        if name=='qr_enable':
            with self.lock:self.qr_enabled=bool(payload.get('enabled'))
            return {'enabled':bool(payload.get('enabled'))}
        raise ValueError('Unknown command')

    def run(self):
        for line in sys.stdin:
            try:
                data=json.loads(line)
                cmd=data['command'];reqid=data['id'];payload=data.get('payload') or {}
                # command can block; never delay stdin reader (STOP priority)
                def respond(command=cmd, request=reqid, args=payload):
                    try:emit('response',id=request,ok=True,result=self.handle(command,args))
                    except Exception as exc:emit('response',id=request,ok=False,error=str(exc))
                threading.Thread(target=respond,daemon=True).start()
            except Exception as exc:emit('diagnostic',message='Command error: '+str(exc))

if __name__=='__main__':
    emit('event',message='ROS adapter starting')
    try:Adapter().run()
    except BaseException as exc:
        emit('diagnostic',message='ROS adapter exited: {}'.format(exc))
        raise
