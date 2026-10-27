"""Deadman-controlled camera-arm jog generator. No ROS dependency.

All browser messages carry monotonically increasing sequence numbers. The server
owns movement; a missing heartbeat or explicit release stops future targets.
An already accepted servo target may complete (hardware-driver limitation).
"""
import threading
import time

class ArmHold:
    KEY_AXES={'a':('pan',-1),'d':('pan',1),'w':('tilt',1),'s':('tilt',-1)}
    def __init__(self, angles, now=None, interval=.18, timeout=.42, degrees_per_sec=8.0):
        self.angles=angles.copy()
        self.clock=now or time.monotonic
        self.interval=interval
        self.timeout=timeout
        self.degrees_per_sec=degrees_per_sec
        self.limits={'pan':(0.0,180.0),'tilt':(0.0,90.0)}
        self.lock=threading.RLock()
        self.key=None
        self.sequence=-1
        self.last_heartbeat=0.0
        self.last_step=0.0
        self.feedback_time={'pan':0.0,'tilt':0.0}

    def observe(self,axis,angle):
        if axis not in self.limits:return
        with self.lock:
            self.angles[axis]=float(angle)
            self.feedback_time[axis]=self.clock()

    def pulse(self,key,sequence):
        if key not in self.KEY_AXES:raise ValueError('Unknown arm jog key')
        with self.lock:
            sequence=int(sequence)
            if sequence<=self.sequence:return False
            self.sequence=sequence
            self.key=key
            self.last_heartbeat=self.clock()
            return True

    def stop(self,sequence=None):
        with self.lock:
            if sequence is not None:
                sequence=int(sequence)
                if sequence<self.sequence:return False
                self.sequence=sequence
            self.key=None
            return True

    def tick(self):
        with self.lock:
            now=self.clock()
            if not self.key:return None
            if now-self.last_heartbeat>self.timeout:
                self.key=None
                return None
            if now-self.last_step<self.interval:return None
            axis,direction=self.KEY_AXES[self.key]
            lo,hi=self.limits[axis]
            new=max(lo,min(hi,self.angles[axis]+direction*self.degrees_per_sec*self.interval))
            self.last_step=now
            if abs(new-self.angles[axis])<1e-7:return None
            self.angles[axis]=new
            return axis,new
