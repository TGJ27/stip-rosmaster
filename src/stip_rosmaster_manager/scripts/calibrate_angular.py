#!/usr/bin/env python3
# coding: utf-8
"""STIP adaptation of Yahboom calibrate_angular.py (Pi Robot Project origins).

Original algorithm and dynamic_reconfigure fields from the supplied Yahboom
archive. GPL-2.0-or-later derivation; see README.
Safety: starts idle, bounded speed/time, fresh TF, zero Twist on all exits.
"""
import math
import time
import threading
import rospy
import tf
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from stip_rosmaster_manager.cfg import CalibrateAngularConfig


def normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class CalibrateAngular:
    def __init__(self):
        rospy.init_node('calibrate_angular')
        self.lock = threading.RLock()
        self.rate = rospy.Rate(20)
        self.max_duration = max(2.0, min(60.0, float(rospy.get_param('~max_test_duration', 18.0))))
        self.max_speed = 0.65
        self.start_test = False
        self.angle = math.pi / 2
        self.speed = 0.35
        self.tolerance = math.radians(1.5)
        self.scale = 1.0
        self.next_direction = 1
        self.tf = tf.TransformListener()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        rospy.on_shutdown(self.stop)
        self.server = Server(CalibrateAngularConfig, self.reconfigure)
        rospy.loginfo('STIP Yahboom angular calibration READY. Use rqt_reconfigure /calibrate_angular; start_test=false by default.')
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
            self.angle = math.radians(min(float(config['test_angle']), 360.0))
            self.speed = min(abs(float(config['speed'])), self.max_speed)
            self.tolerance = math.radians(float(config['tolerance']))
            self.scale = float(config['odom_angular_scale_correction'])
        return config

    def yaw(self):
        try:
            t = self.tf.getLatestCommonTime('/odom', '/base_footprint')
            if t == rospy.Time(0) or (rospy.Time.now() - t).to_sec() > 1.5:
                raise RuntimeError('stale odom TF')
            _, q = self.tf.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
            return float(tf.transformations.euler_from_quaternion(q)[2])
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
        begun, last_yaw, progress, direction = None, None, 0.0, 1
        while not rospy.is_shutdown():
            try:
                with self.lock:
                    enabled, angle, speed = self.start_test, self.angle, self.speed
                    tolerance, scale = self.tolerance, self.scale
                if not enabled:
                    if begun is not None:
                        self.stop()
                        rospy.loginfo('Angular test cancelled.')
                    begun = None
                    self.rate.sleep()
                    continue
                if begun is None:
                    last_yaw = self.yaw()
                    begun, progress = time.monotonic(), 0.0
                    direction = self.next_direction
                    self.next_direction *= -1
                    rospy.logwarn('ANGULAR MOTION STARTED: %.1f degrees, speed <= %.2frad/s', math.degrees(angle), speed)
                if not math.isfinite(scale) or scale <= 0 or speed < 0.05 or angle <= 0:
                    self.finish('Invalid angular calibration parameters, stopped.')
                    begun = None
                    continue
                if time.monotonic() - begun > self.max_duration:
                    self.finish('Angular motion timeout (%.1fs); stopped.' % self.max_duration)
                    begun = None
                    continue
                yaw = self.yaw()
                progress += direction * normalize(yaw-last_yaw)*scale
                last_yaw = yaw
                if progress >= angle - tolerance:
                    self.finish('Angular test finished: %.1f degrees corrected' % math.degrees(progress))
                    begun = None
                    continue
                cmd = Twist()
                cmd.angular.z = direction * speed
                self.pub.publish(cmd)
            except (RuntimeError, ValueError, rospy.ROSInterruptException) as exc:
                self.finish('Angular test stopped: %s' % exc)
                begun = None
            self.rate.sleep()
        self.stop()

if __name__ == '__main__':
    CalibrateAngular()
