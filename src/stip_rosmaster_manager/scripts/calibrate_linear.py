#!/usr/bin/env python3
# coding: utf-8
"""STIP adaptation of Yahboom calibrate_linear.py (Pi Robot Project origins).

Original algorithm and dynamic_reconfigure fields from the user-provided
Yahboom package. See README for provenance and GPL-2.0-or-later notice.
Safety additions: idle startup, bounded duration/speed/distance, TF freshness,
zero velocity on cancellation/timeout/errors/shutdown.
"""
import math
import time
import threading
import rospy
import tf
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from stip_rosmaster_manager.cfg import CalibrateLinearConfig


class CalibrateLinear:
    def __init__(self):
        rospy.init_node('calibrate_linear')
        self.lock = threading.RLock()
        self.rate = rospy.Rate(20)
        self.max_duration = max(2.0, min(60.0, float(rospy.get_param('~max_test_duration', 18.0))))
        self.max_speed = 0.20
        self.max_distance = 1.5
        self.started = False
        self.start_test = False
        self.distance = 1.0
        self.speed = 0.12
        self.tolerance = 0.03
        self.scale = 1.0
        self.direction = 1
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        self.tf = tf.TransformListener()
        rospy.on_shutdown(self.stop)
        self.server = Server(CalibrateLinearConfig, self.reconfigure)
        rospy.loginfo('STIP Yahboom linear calibration READY. Use rqt_reconfigure /calibrate_linear; start_test=false by default.')
        self.run()

    def stop(self):
        try:
            for _ in range(3):
                self.pub.publish(Twist())
                time.sleep(0.025)
        except Exception:
            pass

    def reconfigure(self, config, _level):
        with self.lock:
            self.start_test = bool(config['start_test'])
            self.distance = min(float(config['test_distance']), self.max_distance)
            self.speed = min(abs(float(config['speed'])), self.max_speed)
            self.tolerance = max(float(config['tolerance']), 0.005)
            self.scale = float(config['odom_linear_scale_correction'])
            self.direction = int(config['direction'])
        return config

    def current_position(self):
        try:
            t = self.tf.getLatestCommonTime('/odom', '/base_footprint')
            if t == rospy.Time(0) or (rospy.Time.now() - t).to_sec() > 1.5:
                raise RuntimeError('stale odom TF')
            tr, _ = self.tf.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
            return float(tr[0]), float(tr[1])
        except (tf.Exception, rospy.ROSException) as exc:
            raise RuntimeError('No fresh /odom -> /base_footprint transform: ' + str(exc))

    def finish(self, text):
        self.stop()
        with self.lock:
            self.start_test = False
        rospy.logwarn(text)
        try:
            self.server.update_configuration({'start_test': False})
        except Exception as exc:
            rospy.logwarn('Could not reset start_test: %s', exc)

    def run(self):
        begun, origin = None, None
        while not rospy.is_shutdown():
            try:
                with self.lock:
                    enabled, distance, speed = self.start_test, self.distance, self.speed
                    tolerance, scale, direction = self.tolerance, self.scale, self.direction
                if not enabled:
                    if begun is not None:
                        self.stop()
                        rospy.loginfo('Linear test cancelled.')
                    begun, origin = None, None
                    self.rate.sleep()
                    continue
                if begun is None:
                    origin = self.current_position()
                    begun = time.monotonic()
                    rospy.logwarn('LINEAR MOTION STARTED: distance %.2fm, speed <= %.2fm/s', distance, speed)
                if not math.isfinite(scale) or scale <= 0 or speed < 0.01 or distance <= 0:
                    self.finish('Invalid calibration parameters, test stopped.')
                    begun = None
                    continue
                if time.monotonic() - begun > self.max_duration:
                    self.finish('Linear motion timeout (%.1fs); stopped.' % self.max_duration)
                    begun = None
                    continue
                x, y = self.current_position()
                measured = math.hypot(x-origin[0], y-origin[1]) * scale
                if measured >= distance - tolerance:
                    self.finish('Linear test finished, odom measured %.3fm (corrected)' % measured)
                    begun = None
                    continue
                cmd = Twist()
                if direction == 1:
                    cmd.linear.x = speed
                else:
                    cmd.linear.y = speed
                self.pub.publish(cmd)
            except (RuntimeError, ValueError, rospy.ROSInterruptException) as exc:
                self.finish('Linear test stopped: %s' % exc)
                begun = None
            self.rate.sleep()
        self.stop()

if __name__ == '__main__':
    CalibrateLinear()
