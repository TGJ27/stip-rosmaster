"""Supervises ROS worker subprocess and manages request/response transport."""
import base64
import collections
import json
import queue
import subprocess
import sys
import threading
import time
from pathlib import Path
import os
from camera_feed import CameraFeed
HERE=Path(__file__).resolve().parent

class ROSBridge:
    def __init__(self):
        self.proc = None
        self.lock = threading.RLock()
        self.pending = {}
        self.nextid = 0
        self.state = {'connected': False, 'error': 'ROS adapter not connected'}
        self.live_map = None
        self.map_meta = None
        # Unique IPC file for this dashboard instance, cleaned up on server exit.
        self.camera_path=Path('/dev/shm' if Path('/dev/shm').is_dir() else '/tmp') / ('stip_camera_%d_%d.jpg' % (os.getuid(), os.getpid()))
        self.camera_feed=CameraFeed(self.camera_path)
        self.registered_path=self.camera_path.with_name(self.camera_path.stem+'_registered.jpg')
        self.registered_feed=CameraFeed(self.registered_path,max_age=1.8)
        self._camera_fallback=None
        self.thermal = None
        self.events = collections.deque(maxlen=90)
        self.last_heard = 0
        self.stopping = False
        threading.Thread(target=self.supervise, daemon=True).start()

    @property
    def camera(self):
        data=self.camera_feed.read()
        if data:return data
        # Legacy adapter compatibility, if direct-file publisher is unavailable.
        if self.last_heard and time.monotonic()-self.last_heard<3:
            return self._camera_fallback
        return None

    @property
    def registered_camera(self):
        return self.registered_feed.read()

    def supervise(self):
        while not self.stopping:
            try:
                self.proc = subprocess.Popen([sys.executable, '-u', str(HERE/'ros_worker.py')],
                                             stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                             stderr=subprocess.PIPE, bufsize=1, text=True,
                                             env=dict(os.environ,STIP_DASHBOARD_CAMERA_PATH=str(self.camera_path),
                                                      STIP_DASHBOARD_REGISTERED_PATH=str(self.registered_path)))
                threading.Thread(target=self.read_errors, args=(self.proc,), daemon=True).start()
                for line in self.proc.stdout:
                    try:
                        data=json.loads(line)
                    except ValueError:
                        # rospy/rosconsole may print ordinary log lines to stdout.
                        with self.lock:self.events.append('ROS: '+line.rstrip()[:250])
                        continue
                    with self.lock:
                        kind=data.get('kind')
                        self.last_heard=time.monotonic()
                        if kind=='state':self.state=data['data'];self.state['connected']=bool(data['data'].get('connected',False))
                        elif kind=='camera':self._camera_fallback=base64.b64decode(data['jpeg'])
                        elif kind=='thermal':self.thermal=base64.b64decode(data['jpeg'])
                        elif kind=='map':
                            self.live_map=base64.b64decode(data['image']);self.map_meta=data['meta']
                        elif kind=='response':
                            slot=self.pending.get(data['id'])
                            if slot:slot.put(data)
                        elif kind in ('event','diagnostic'):self.events.append(data.get('message',''))
            except Exception as exc:
                with self.lock:self.events.append('ROS adapter unavailable: {}'.format(exc))
                try:
                    if self.proc and self.proc.poll() is None:
                        self.proc.terminate()
                except Exception:pass
            finally:
                with self.lock:
                    self.state={'connected':False,'error':'ROS adapter offline; dashboard is still available'}
                    for req in self.pending.values():
                        try:req.put_nowait({'ok':False,'error':'ROS adapter disconnected'})
                        except queue.Full:pass
                    self.pending.clear()
            time.sleep(3)

    def read_errors(self,p):
        try:
            for line in p.stderr:
                with self.lock:self.events.append(line.rstrip()[:240])
        except Exception:pass

    def command(self, name, payload=None, timeout=6):
        with self.lock:
            if not self.proc or self.proc.poll() is not None:
                raise ValueError('ROS adapter unavailable; check ROS master and Python environment')
            self.nextid+=1
            reqid=self.nextid
            result=queue.Queue(maxsize=1)
            self.pending[reqid]=result
            try:
                self.proc.stdin.write(json.dumps({'id':reqid,'command':name,'payload':payload or {}})+'\n')
                self.proc.stdin.flush()
            except Exception as exc:
                self.pending.pop(reqid,None)
                raise ValueError('ROS adapter write failed: {}'.format(exc))
        try:
            answer=result.get(timeout=timeout)
            if not answer.get('ok'):raise ValueError(answer.get('error','ROS operation failed'))
            return answer.get('result',{})
        except queue.Empty:
            raise ValueError('ROS operation timed out; check component health')
        finally:
            with self.lock:self.pending.pop(reqid,None)

