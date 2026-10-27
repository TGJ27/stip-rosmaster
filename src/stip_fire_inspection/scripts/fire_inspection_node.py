#!/usr/bin/env python3
"""STIP camera-arm fire investigation. Existing detectors, drivers remain untouched.

Only existing RGB detections /fire_detection/detections and native 32FC1
/thermal/image_raw enter the decision. No dashboard, no autonomous navigation.
"""
import json
import math
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rospy
import rosgraph
import rospkg
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Float64MultiArray, String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from stip_rosmaster_msgs.msg import ArmJoint
from stip_rosmaster_msgs.srv import RobotArmArray

# Catkin devel-space entrypoints may be wrappers in devel/lib; resolve actual source.
sys.path.insert(0, os.path.join(rospkg.RosPack().get_path('stip_fire_inspection'), 'scripts'))
from fusion_core import (analyze_target, control_step, load_homography,
                         parse_detections, passes_temperature, thermal_center_rgb)


class FireInspection:
    def __init__(self):
        p = lambda name, default: rospy.get_param('~' + name, default)
        self.alignment_file = os.path.expanduser(p('alignment_file',''))
        if not os.path.isfile(self.alignment_file):
            raise RuntimeError('Missing saved thermal alignment: %s. Run thermal RGB calibration first.' % self.alignment_file)
        with open(self.alignment_file) as f:
            self.h, self.refw, self.refh = load_homography(yaml.safe_load(f))
        self.min_conf = float(p('min_ai_confidence', .40))
        self.ai_min = max(2,int(p('min_unique_ai_frames',2)))
        self.ai_age_limit = float(p('max_ai_age_sec', .80))
        self.hold = float(p('candidate_hold_sec',1.5))
        self.thermal_age = float(p('max_thermal_age_sec',.6))
        self.pair_age = float(p('max_pair_age_sec',.35))
        self.verify_timeout = float(p('verify_timeout_sec',8.0))
        self.required_hot = max(2,int(p('required_hot_frames',3)))
        self.hot_min = float(p('min_hot_c',65))
        self.delta_min = float(p('min_above_background_c',15))
        self.min_roi_pixels = max(1, int(p('min_roi_pixels',3)))
        self.min_hot_pixels = max(2, int(p('min_hot_pixels',2)))
        self.top_percentile = float(p('thermal_top_percentile',90))
        self.aim_enabled = bool(p('auto_aim', False))
        self.stationary_mode = bool(p('stationary_test_mode',False))
        self.require_pause = bool(p('require_motion_pause',True))
        self.exclusive = bool(p('arm_exclusive',True))
        self.pan_id = int(p('pan_servo_id',1))
        self.tilt_id = int(p('tilt_servo_id',2))
        self.pan_min = float(p('pan_min',60))
        self.pan_max = float(p('pan_max',120))
        self.tilt_min = float(p('tilt_min',5))
        self.tilt_max = float(p('tilt_max',60))
        self.pan_sign = float(p('pan_sign',1))
        self.tilt_sign = float(p('tilt_sign',-1))
        self.pan_gain = float(p('pan_gain_deg',2))
        self.tilt_gain = float(p('tilt_gain_deg',2))
        self.max_step = float(p('max_servo_step_deg',1.5))
        self.deadband = float(p('center_deadband_fraction',.12))
        self.servo_time = int(p('servo_run_time_ms',180))
        self.command_interval = float(p('min_servo_interval_sec',.35))
        self.settle = float(p('servo_settle_sec',.35))
        self.max_aim_time = float(p('max_aim_sec',8.0))
        # Smooth AI target jitter without introducing long temporal delay.
        self.target_filter_alpha = float(p('target_filter_alpha',0.55))
        # Fire must disappear from fresh inference results AND the last thermal
        # target must cool before this controller relinquishes its alarm.
        self.alarm_clear_sec = float(p('alarm_clear_sec',4.0))
        self.alarm_clear_frames = max(2, int(p('alarm_clear_unique_ai_frames',8)))
        self.alarm_clear_cool_frames = max(2, int(p('alarm_clear_cool_frames',3)))
        self.alarm_clear_below_c = float(p('alarm_clear_below_c',60.0))
        self.buzzer_refresh_sec = float(p('buzzer_refresh_sec',1.0))
        self.pop_buzzer = bool(p('enable_pop32i_buzzer',False))
        self.buzzer_topic = p('buzzer_command_topic','/gas_sensor/alarm_cmd')
        if not (0 < self.ai_age_limit <= 3 and 0 < self.thermal_age <= 3 and
                0 < self.pair_age <= 2 and 0 < self.verify_timeout <= 60 and
                0 < self.max_aim_time <= 60 and 0. <= self.deadband < .5 and
                50 <= self.servo_time <= 1500 and 0 < self.max_step <= 3 and
                self.pan_min < self.pan_max and self.tilt_min < self.tilt_max and
                0 < self.top_percentile <= 100 and
                0 < self.target_filter_alpha <= 1.0 and
                1 <= self.alarm_clear_sec <= 60 and
                -40 < self.alarm_clear_below_c < self.hot_min and
                0.2 <= self.buzzer_refresh_sec <= 10.0):
            raise ValueError('Invalid fire inspection parameters')
        self.lock = threading.RLock()
        self.bridge = CvBridge()
        self.state = 'STARTING'
        self.ai_received = 0.0
        self.ai_unique_recv = 0.0
        self.last_result = None
        self.ai_first = 0.0
        self.ai_last = 0.0
        self.smoke_last = 0.0
        self.ai_stamp = None
        self.ai_streak = 0
        self.target = None
        self.smoothed_center = None
        self.no_fire_since = 0.0
        self.no_fire_frames = 0
        self.clear_ready_since = 0.0
        self.last_clear_thermal_seq = -1
        self.clear_cool_frames = 0
        self.buzzer_owned = False  # only send OFF for alarms we commanded ON
        self.buzzer_off_pending = False
        self.last_target = None
        self.last_target_details = None
        self.thermal_frame = None
        self.thermal_stamp = 0.0
        self.thermal_recv = 0.0
        self.thermal_seq = 0
        self.thermal_healthy = False
        self.had_thermal_health = False
        self.last_examined_seq = -1
        self.good_hot_frames = 0
        self.last_aim_cmd = 0.0
        self.aim_started = 0.0
        self.verification_started = 0.0
        self.aimed_since = 0.0
        self.pose_known = False
        self.pan = None
        self.tilt = None
        self.pose_feedback = 0.0
        self.last_pose_attempt = 0.0
        self.pause_ack = False
        self.moving_recently = 0.0
        self.hardware_ack = False
        self.last_buzzer_pub = 0.0
        self.alarm_latched = False
        self.warned_no_arm = False
        self.last_diag = 0.0
        self.hot_stats = None
        self.warning_pub = rospy.Publisher('/fire_inspection/warning',Bool,queue_size=2,latch=True)
        self.confirmed_pub = rospy.Publisher('/fire_inspection/confirmed',Bool,queue_size=2,latch=True)
        self.alarm_pub = rospy.Publisher('/fire_inspection/alarm_requested',Bool,queue_size=2,latch=True)
        self.state_pub = rospy.Publisher('/fire_inspection/state',String,queue_size=2,latch=True)
        self.status_pub = rospy.Publisher('/fire_inspection/status_json',String,queue_size=2,latch=True)
        self.temp_pub = rospy.Publisher('/fire_inspection/target_hot_c',Float32,queue_size=2)
        self.ambient_pub = rospy.Publisher('/fire_inspection/background_c',Float32,queue_size=2)
        self.target_pub = rospy.Publisher('/fire_inspection/target_rgb',Float64MultiArray,queue_size=2)
        self.pause_pub = rospy.Publisher('/fire_inspection/request_navigation_pause',Bool,queue_size=2,latch=True)
        self.arm_pub = rospy.Publisher('/TargetAngle',ArmJoint,queue_size=2)
        self.buzzer_pub = rospy.Publisher(self.buzzer_topic,Bool,queue_size=2)
        rospy.Subscriber(p('fire_topic','/fire_detection/detections'),Float64MultiArray,self.on_detection,queue_size=1)
        rospy.Subscriber(p('thermal_topic','/thermal/image_raw'),Image,self.on_thermal,queue_size=1,buff_size=256*1024)
        rospy.Subscriber(p('thermal_health_topic','/thermal/healthy'),Bool,self.on_thermal_health,queue_size=1)
        rospy.Subscriber('/ArmAngleUpdate',ArmJoint,self.on_arm,queue_size=5)
        rospy.Subscriber('/fire_inspection/navigation_paused',Bool,self.on_pause,queue_size=1)
        rospy.Subscriber('/cmd_vel',Twist,self.on_velocity,queue_size=3)
        rospy.Subscriber('/gas_sensor/alarm_state',Bool,self.on_buzzer_state,queue_size=2)
        rospy.Service('/fire_inspection/set_auto_aim',SetBool,self.set_aim)
        rospy.Service('/fire_inspection/acknowledge',Trigger,self.acknowledge)
        rospy.Timer(rospy.Duration(.1),self.tick)
        rospy.on_shutdown(self.shutdown)
        self.publish_status()
        rospy.logwarn('FIRE INSPECTION: standalone experimental verifier; NO navigation pause implemented here.')
        rospy.logwarn('AUTO ARM %s, POP32i BUZZER %s. All fire warnings remain ROS-visible.',
                      'ON' if self.aim_enabled else 'OFF', 'ON' if self.pop_buzzer else 'OFF')

    def on_detection(self, msg):
        d = parse_detections(msg.data,self.ai_age_limit,self.min_conf)
        now = time.monotonic()
        if d is None:
            return
        with self.lock:
            self.last_result = d
            self.ai_received = now
            # A cached visualization row can recur at 30 FPS. Count inference once.
            stamp = d['stamp']
            if self.ai_stamp is not None and abs(stamp-self.ai_stamp)<1e-5:
                return
            self.ai_stamp = stamp
            self.ai_unique_recv = now
            choices = [x for x in d['boxes'] if x['class_id']==0]
            if not choices:
                # Count only NEW AI inference results; never treat stale camera
                # silence as evidence that the fire has disappeared.
                if self.no_fire_since <= 0.0:
                    self.no_fire_since = now
                self.no_fire_frames += 1
                # Smoke never directly confirms a fire by temperature.
                if any(x['class_id']==1 for x in d['boxes']):
                    self.last_target_details = 'smoke'
                    self.smoke_last = now
                else:
                    self.last_target_details = None
                self.ai_streak = 0
                return
            self.no_fire_since = 0.0
            self.no_fire_frames = 0
            self.clear_ready_since = 0.0
            self.clear_cool_frames = 0
            if self.target and self.ai_last and now-self.ai_last < self.hold:
                old = self.target['center']
                # Prefer maintaining target identity instead of chasing the next highest score.
                choices.sort(key=lambda x: ((x['center'][0]-old[0])/d['width'])**2 +
                                               ((x['center'][1]-old[1])/d['height'])**2 -
                                               .01*x['confidence'])
            else:
                choices.sort(key=lambda x: -x['confidence'])
            chosen = choices[0]
            close = (self.target is not None and
                     abs(chosen['center'][0]-self.target['center'][0]) < .22*d['width'] and
                     abs(chosen['center'][1]-self.target['center'][1]) < .22*d['height'] and
                     now-self.ai_last < self.hold)
            self.ai_streak = self.ai_streak+1 if close else 1
            if not close:
                self.good_hot_frames = 0
                self.hot_stats = None
                self.last_examined_seq = -1
                self.aim_started = now
                self.verification_started = now
            self.target = chosen
            xy = chosen['center']
            if not close or self.smoothed_center is None:
                self.smoothed_center = (float(xy[0]), float(xy[1]))
            else:
                a = self.target_filter_alpha
                self.smoothed_center = (a*xy[0]+(1-a)*self.smoothed_center[0],
                                        a*xy[1]+(1-a)*self.smoothed_center[1])
            self.last_target = d
            self.last_target_details = 'fire'
            self.ai_last = now
            self.ai_first = self.ai_first if close else now

    def on_thermal(self, msg):
        try:
            frame = np.asarray(self.bridge.imgmsg_to_cv2(msg,desired_encoding='32FC1'),dtype=np.float32)
            if frame.shape != (24,32) or not np.isfinite(frame).all() or frame.min() < -40 or frame.max() > 300:
                return
            now = time.monotonic()
            with self.lock:
                self.thermal_frame = frame.copy()
                self.thermal_recv = now
                self.thermal_stamp = msg.header.stamp.to_sec() if msg.header.stamp != rospy.Time(0) else 0.0
                self.thermal_seq += 1
        except (ValueError,RuntimeError,TypeError) as exc:
            rospy.logwarn_throttle(3,'Fire inspection temperature decode: %s',exc)

    def on_thermal_health(self,msg):
        with self.lock:
            self.had_thermal_health = True
            self.thermal_healthy = bool(msg.data)

    def on_arm(self,msg):
        with self.lock:
            if msg.id==self.pan_id:
                self.pan=float(msg.angle)
                self.pose_known=True
                self.pose_feedback=time.monotonic()
            elif msg.id==self.tilt_id:
                self.tilt=float(msg.angle)
                self.pose_known=True
                self.pose_feedback=time.monotonic()

    def on_pause(self,msg):
        with self.lock:
            self.pause_ack = bool(msg.data)

    def on_velocity(self,msg):
        if abs(msg.linear.x)>.015 or abs(msg.linear.y)>.015 or abs(msg.angular.z)>.025:
            with self.lock:
                self.moving_recently=time.monotonic()

    def on_buzzer_state(self,msg):
        with self.lock:
            self.hardware_ack = bool(msg.data)

    def set_aim(self,req):
        with self.lock:
            if req.data and self.require_pause and not (self.stationary_mode or self.pause_ack):
                return SetBoolResponse(False,'Arm enable blocked: no stationary-mode acknowledgement or navigation-paused signal')
            self.aim_enabled=bool(req.data)
            return SetBoolResponse(True,'Auto aiming '+('ENABLED' if self.aim_enabled else 'DISABLED'))

    def acknowledge(self,_request):
        with self.lock:
            if self.target is not None and time.monotonic()-self.ai_last < self.hold:
                return TriggerResponse(False,'Fire detection still present; cannot acknowledge')
            self.release_alarm('MANUAL_CLEAR')
            rospy.logwarn('Fire alarm manually acknowledged after RGB fire detections cleared')
            return TriggerResponse(True,'Fire alarm acknowledged; POP32i OFF requested if this controller owns the buzzer')

    def get_arm_pose(self,now):
        if self.pan is not None and self.tilt is not None and self.pose_known:
            return True
        if now-self.last_pose_attempt<2.0:
            return False
        self.last_pose_attempt=now
        try:
            rospy.wait_for_service('/CurrentAngle',timeout=.25)
            angles=rospy.ServiceProxy('/CurrentAngle',RobotArmArray)('get').angles
            i,j=self.pan_id-1,self.tilt_id-1
            if min(i,j)>=0 and max(i,j)<len(angles):
                if math.isfinite(angles[i]) and math.isfinite(angles[j]) and angles[i]>=0 and angles[j]>=0:
                    self.pan=float(angles[i]);self.tilt=float(angles[j]);self.pose_known=True
                    self.pose_feedback=now
                    return True
        except (rospy.ROSException,rospy.ServiceException,Exception) as exc:
            rospy.logwarn_throttle(5,'Fire inspection arm pose unavailable: %s',exc)
        return False

    def owns_arm(self):
        if not self.exclusive:
            return True
        try:
            pubs,_,_=rosgraph.Master(rospy.get_name()).getSystemState()
            for topic, nodes in pubs:
                if topic=='/TargetAngle':
                    competing=[name for name in nodes if name != rospy.get_name()]
                    return not competing
        except Exception:
            return False  # fail closed
        return True

    def send_axis(self,servo_id,angle):
        m=ArmJoint()
        m.id=int(servo_id)
        m.angle=float(angle)
        m.run_time=self.servo_time
        m.joints=[]
        self.arm_pub.publish(m)

    def can_aim(self,now):
        if not self.aim_enabled:
            return False
        if self.require_pause and not (self.stationary_mode or self.pause_ack):
            return False
        if now-self.moving_recently<2.0:
            return False
        if not self.owns_arm():
            rospy.logwarn_throttle(5,'Fire auto-aim blocked: another node owns /TargetAngle (QR/UI/teleop?)')
            return False
        return (self.get_arm_pose(now) and self.pan is not None and self.tilt is not None)

    def aim(self,target,source,now):
        if not self.can_aim(now) or now-self.last_aim_cmd<self.command_interval:
            return False
        try:
            center=thermal_center_rgb(self.h,self.refw,self.refh,source['width'],source['height'])
        except ValueError:
            return False
        track_center = self.smoothed_center if self.smoothed_center is not None else target['center']
        errx=track_center[0]-center[0]
        erry=track_center[1]-center[1]
        dx=control_step(errx,source['width'],self.pan_gain,self.max_step,self.deadband,self.pan_sign)
        dy=control_step(erry,source['height'],self.tilt_gain,self.max_step,self.deadband,self.tilt_sign)
        if dx==0.0 and dy==0.0:
            # A new movement away from center starts a fresh bounded aiming run.
            self.aim_started = now
            return True
        if now-self.aim_started>self.max_aim_time:
            self.state='AIM_TIMEOUT'
            return False
        moved=False
        if dx!=0 and self.pan is not None:
            angle=min(self.pan_max,max(self.pan_min,self.pan+dx))
            if abs(angle-self.pan)>.001:
                self.send_axis(self.pan_id,angle)
                self.pan=angle
                moved=True
        if dy!=0 and self.tilt is not None:
            angle=min(self.tilt_max,max(self.tilt_min,self.tilt+dy))
            if abs(angle-self.tilt)>.001:
                self.send_axis(self.tilt_id,angle)
                self.tilt=angle
                moved=True
        if moved:
            self.last_aim_cmd=now
            self.good_hot_frames=0
            self.last_examined_seq=-1
        else:
            self.state='AIM_LIMIT'
        return False

    def publish_status(self):
        is_warning=self.state!='CLEAR' or self.alarm_latched
        self.warning_pub.publish(Bool(data=is_warning))
        self.confirmed_pub.publish(Bool(data=self.alarm_latched))
        self.alarm_pub.publish(Bool(data=self.alarm_latched))
        self.pause_pub.publish(Bool(data=is_warning))
        self.state_pub.publish(String(data=self.state))
        self.status_pub.publish(String(data=json.dumps({
            'state':self.state,'ai_unique_frames':self.ai_streak,
            'hot_frames':self.good_hot_frames,'auto_aim':self.aim_enabled,
            'arm_pose_known':self.pose_known,'stationary_test_mode':self.stationary_mode,
            'navigation_pause_ack':self.pause_ack,'alarm_latched':self.alarm_latched,
            'no_fire_frames':self.no_fire_frames,
            'cool_thermal_frames':self.clear_cool_frames,
            'ai_unique_age_sec':round(time.monotonic()-self.ai_unique_recv,2) if self.ai_unique_recv else None,
            'alarm_clear_sec':self.alarm_clear_sec,
            'buzzer_owned':self.buzzer_owned,
            'target_filter_alpha':self.target_filter_alpha,
            'pop32i_buzzer_enabled':self.pop_buzzer,'buzzer_ack':self.hardware_ack,
            'roi_temperature':self.hot_stats,
        },sort_keys=True)))

    def command_buzzer(self, on, now):
        if not self.pop_buzzer:
            return
        if on:
            self.buzzer_owned = True
            self.buzzer_off_pending = False
        elif not self.buzzer_owned:
            return  # never silence an independently asserted gas/external alarm
        self.buzzer_pub.publish(Bool(data=bool(on)))
        self.last_buzzer_pub = now
        if not on:
            self.buzzer_off_pending = True
        rospy.logwarn('POP32i fire alarm command: %s', 'ON' if on else 'OFF')

    def release_alarm(self, new_state='CLEAR'):
        """Clear only this package's confirmed-fire alarm and rearm tracking."""
        now = time.monotonic()
        self.alarm_latched = False
        if self.buzzer_owned:
            self.command_buzzer(False, now)
        self.state = new_state
        self.good_hot_frames = 0
        self.hot_stats = None
        self.target = None
        self.smoothed_center = None
        self.ai_streak = 0
        self.aim_started = 0.0
        self.verification_started = 0.0
        self.last_examined_seq = -1
        self.no_fire_since = 0.0
        self.no_fire_frames = 0
        self.clear_ready_since = 0.0
        self.last_clear_thermal_seq = -1
        self.clear_cool_frames = 0
        self.publish_status()

    def refresh_buzzer(self, now):
        if not self.pop_buzzer:
            return
        if self.alarm_latched:
            # The serial bridge can restart while the alarm is active.
            if not self.hardware_ack and now-self.last_buzzer_pub >= self.buzzer_refresh_sec:
                self.command_buzzer(True, now)
        elif self.buzzer_off_pending:
            # Repeat OFF briefly until the bridge acknowledges it.
            if not self.hardware_ack and now-self.last_buzzer_pub >= 2.0:
                self.buzzer_off_pending = False
                self.buzzer_owned = False
            elif now-self.last_buzzer_pub >= self.buzzer_refresh_sec:
                self.command_buzzer(False, now)

    def tick_confirmed(self, now):
        """Remain tracking while confirmed; clear only on fresh RGB + cool thermal."""
        self.refresh_buzzer(now)
        ai_online = self.ai_unique_recv > 0 and now-self.ai_unique_recv <= self.ai_age_limit
        thermal_online = (self.thermal_frame is not None and
                          now-self.thermal_recv <= self.thermal_age and
                          (not self.had_thermal_health or self.thermal_healthy))
        fire_visible = (self.target is not None and
                        now-self.ai_last < self.hold and self.no_fire_frames == 0)
        self.state = 'CONFIRMED_ACTIVE'
        if fire_visible:
            # Do not freeze camera-arm tracking after the first confirmation.
            if self.aim_enabled and self.last_target is not None:
                try:
                    center=thermal_center_rgb(self.h,self.refw,self.refh,
                                              self.last_target['width'],self.last_target['height'])
                    xy=self.smoothed_center or self.target['center']
                    ex=xy[0]-center[0]
                    ey=xy[1]-center[1]
                    centered=(abs(ex)/(self.last_target['width']*.5)<=self.deadband and
                              abs(ey)/(self.last_target['height']*.5)<=self.deadband)
                    if not centered:
                        self.aim(self.target,self.last_target,now)
                    else:
                        self.aim_started=now
                except ValueError:
                    self.state='CONFIRMED_AIM_UNAVAILABLE'
            self.publish_status()
            return
        # An AI stream dropout or thermal failure must NEVER become an auto-clear.
        if not ai_online or not thermal_online:
            self.state='CONFIRMED_SENSOR_UNVERIFIED'
            self.clear_ready_since=0.0
            self.clear_cool_frames=0
            self.publish_status()
            return
        if self.last_target_details == 'smoke' and now-self.smoke_last < self.hold:
            self.state='CONFIRMED_SMOKE_REMAINS'
            self.clear_ready_since=0.0
            self.clear_cool_frames=0
            self.publish_status()
            return
        if (self.no_fire_since<=0 or self.no_fire_frames < self.alarm_clear_frames or
                now-self.no_fire_since < self.alarm_clear_sec):
            self.state='CONFIRMED_WAIT_CLEAR'
            self.publish_status()
            return
        if self.last_target is None or self.target is None:
            self.state='CONFIRMED_TARGET_UNAVAILABLE'
            self.publish_status()
            return
        if self.thermal_seq != self.last_clear_thermal_seq:
            self.last_clear_thermal_seq = self.thermal_seq
            stats=analyze_target(self.thermal_frame,self.target['box'],self.h,self.refw,self.refh,
                                 self.last_target['width'],self.last_target['height'],
                                 self.min_roi_pixels,self.top_percentile,self.hot_min)
            if stats is None:
                self.clear_ready_since = 0.0
                self.clear_cool_frames = 0
                self.state='CONFIRMED_CLEAR_TEMPERATURE_UNAVAILABLE'
            elif stats['hot_c'] >= self.alarm_clear_below_c:
                self.clear_ready_since = 0.0
                self.clear_cool_frames = 0
                self.state='CONFIRMED_WAIT_COOL'
                self.hot_stats=stats
            else:
                if self.clear_ready_since <= 0:
                    self.clear_ready_since=now
                self.clear_cool_frames += 1
                self.state='CONFIRMED_COOLING_VERIFIED'
                self.hot_stats=stats
        if (self.clear_ready_since > 0 and self.clear_cool_frames >= self.alarm_clear_cool_frames
                and now-self.clear_ready_since >= 0.5):
            rospy.logwarn('FIRE CLEARED: %.1fs without RGB fire and thermal target cooled below %.1f C; releasing alarm',
                          now-self.no_fire_since,self.alarm_clear_below_c)
            self.release_alarm('CLEAR')
        else:
            self.publish_status()

    def tick(self,_event):
        with self.lock:
            now=time.monotonic()
            if self.alarm_latched:
                self.tick_confirmed(now)
                return
            self.refresh_buzzer(now)
            fire_fresh=(self.target is not None and self.ai_streak>=self.ai_min and now-self.ai_last<self.hold
                        and self.no_fire_frames==0)
            smoke=(self.last_target_details=='smoke' and self.last_result and
                   self.last_result['boxes'] and now-self.smoke_last<self.hold and not fire_fresh)
            if not fire_fresh:
                if smoke:
                    self.state='SMOKE_UNVERIFIED'
                elif self.target is not None and now-self.ai_last<self.hold:
                    self.state='SUSPECTED'
                elif self.ai_received <= 0 or now-self.ai_received > 2.0:
                    self.state='UNVERIFIED_AI_OFFLINE'
                elif (self.had_thermal_health and not self.thermal_healthy) or (
                        self.thermal_frame is None or now-self.thermal_recv>2.0):
                    self.state='UNVERIFIED_THERMAL_OFFLINE'
                else:
                    self.state='CLEAR'
                    self.target=None
                    self.smoothed_center=None
                    self.good_hot_frames=0
                    self.hot_stats=None
                self.publish_status()
                return
            self.state='SUSPECTED'
            source=self.last_target
            if source is None:
                self.publish_status()
                return
            self.target_pub.publish(Float64MultiArray(data=[float(v) for v in self.target['box']]))
            if now-self.verification_started>self.verify_timeout:
                self.state='UNVERIFIED_TIMEOUT'
                self.publish_status()
                return
            # Require target centered in saved thermal optical frame.
            if self.aim_enabled:
                center=thermal_center_rgb(self.h,self.refw,self.refh,source['width'],source['height'])
                xy=self.smoothed_center or self.target['center']
                ex=xy[0]-center[0];ey=xy[1]-center[1]
                centered=(abs(ex)/(source['width']*.5)<=self.deadband and
                          abs(ey)/(source['height']*.5)<=self.deadband)
                if not centered:
                    self.state='AIMING'
                    self.aim(self.target,source,now)
                    self.publish_status()
                    return
                self.aim_started=now
                if now-self.last_aim_cmd<self.settle:
                    self.state='SETTLING'
                    self.publish_status()
                    return
            if self.had_thermal_health and not self.thermal_healthy:
                self.state='UNVERIFIED_THERMAL_UNHEALTHY'
                self.publish_status()
                return
            if self.thermal_frame is None or now-self.thermal_recv>self.thermal_age:
                self.state='UNVERIFIED_THERMAL_STALE'
                self.publish_status()
                return
            if now-self.ai_last>self.ai_age_limit:
                self.state='UNVERIFIED_AI_STALE'
                self.publish_status()
                return
            if self.thermal_seq==self.last_examined_seq:
                self.state='VERIFYING'
                self.publish_status()
                return
            self.last_examined_seq=self.thermal_seq
            if self.thermal_stamp > 0 and abs(self.thermal_stamp-source['stamp'])>self.pair_age:
                self.state='WAITING_SYNC'
                self.publish_status()
                return
            roi=analyze_target(self.thermal_frame,self.target['box'],self.h,self.refw,self.refh,
                               source['width'],source['height'],self.min_roi_pixels,self.top_percentile,self.hot_min)
            if roi is None:
                self.good_hot_frames=0
                self.state='UNVERIFIED_TARGET_OUTSIDE_THERMAL_FOV'
                self.publish_status()
                return
            self.hot_stats=roi
            self.temp_pub.publish(Float32(data=roi['hot_c']))
            self.ambient_pub.publish(Float32(data=roi['background_c']))
            if passes_temperature(roi,self.hot_min,self.delta_min,self.min_hot_pixels):
                self.good_hot_frames+=1
                self.state='VERIFYING_HOT'
                if self.good_hot_frames>=self.required_hot:
                    self.alarm_latched=True
                    self.state='CONFIRMED_ACTIVE'
                    rospy.logerr('FIRE CONFIRMED (experimental): target hot %.1f C, background %.1f C',
                                 roi['hot_c'],roi['background_c'])
                    self.command_buzzer(True,now)
            else:
                self.good_hot_frames=0
                self.state='VERIFYING_NOT_HOT'
            self.publish_status()

    def shutdown(self):
        # Do not silence a potentially real fire when the ROS process exits.
        rospy.logwarn('Fire inspection shutting down. Hardware alarm state retained; clear only when safe.')


if __name__=='__main__':
    rospy.init_node('stip_fire_inspection')
    try:
        FireInspection()
        rospy.spin()
    except Exception as exc:
        rospy.logerr('STIP fire inspection failed: %s',exc)
        raise SystemExit(1)
