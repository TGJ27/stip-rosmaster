#!/usr/bin/env python3
"""Independent MQ-6 clean-air ADC baseline calibration GUI (ROS Noetic).

Uses the existing gas_bridge Trigger service; never opens USB serial itself,
never writes POP32i firmware and never claims calibrated ppm or %LEL.
"""
from collections import deque
import threading
import time

import cv2
import numpy as np
import rospy
from std_msgs.msg import Bool, Float32, Int32, String
from std_srvs.srv import Trigger


class MQ6BaselineGUI:
    def __init__(self):
        self.lock = threading.Lock()
        self.history = deque(maxlen=180)
        self.raw = None
        self.baseline = None
        self.relative = None
        self.healthy = False
        self.status = 'WAITING FOR SENSOR'
        self.cal_status = 'WAITING'
        self.last_raw_at = 0.0
        self.confirm = False
        self.service_busy = False
        self.note = 'Warm up in known clean air, then click CALIBRATE.'
        self.note_until = time.monotonic() + 20
        self.window = 'STIP MQ-6 Clean-Air Baseline | Separate from Thermal'
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window, self.on_mouse)
        rospy.Subscriber('/gas_sensor/raw', Int32, self.on_raw, queue_size=1)
        rospy.Subscriber('/gas_sensor/baseline', Int32,
                         lambda m: self.set_value('baseline', m.data), queue_size=1)
        rospy.Subscriber('/gas_sensor/change_percent', Float32,
                         lambda m: self.set_value('relative', m.data), queue_size=1)
        rospy.Subscriber('/gas_sensor/healthy', Bool,
                         lambda m: self.set_value('healthy', m.data), queue_size=1)
        rospy.Subscriber('/gas_sensor/status', String,
                         lambda m: self.set_value('status', m.data), queue_size=1)
        rospy.Subscriber('/gas_sensor/calibration_status', String,
                         lambda m: self.set_value('cal_status', m.data), queue_size=1)

    def set_value(self, field, value):
        with self.lock:
            setattr(self, field, value)

    def on_raw(self, msg):
        with self.lock:
            self.raw = int(msg.data)
            self.last_raw_at = time.monotonic()
            self.history.append(int(msg.data))

    def notify(self, message, seconds=10):
        with self.lock:
            self.note = str(message)
            self.note_until = time.monotonic() + seconds
        rospy.loginfo('%s', message)

    def on_mouse(self, event, x, y, _flags, _userdata):
        if event == cv2.EVENT_LBUTTONUP and 30 <= x <= 730 and 405 <= y <= 463:
            self.confirm = True
            self.notify('Confirm known clean air: press Y; press N to cancel.', 30)

    def start_calibration(self):
        if self.service_busy:
            return
        self.service_busy = True
        self.confirm = False
        self.notify('Requesting baseline collection...', 10)

        def run_service():
            try:
                rospy.wait_for_service('/gas_sensor/calibrate', timeout=5)
                res = rospy.ServiceProxy('/gas_sensor/calibrate', Trigger)()
                self.notify(('COLLECTING: ' if res.success else 'NOT STARTED: ') + res.message, 20)
            except (rospy.ROSException, rospy.ServiceException) as error:
                self.notify('CALIBRATION ERROR: {}'.format(error), 20)
            finally:
                self.service_busy = False

        threading.Thread(target=run_service, daemon=True).start()

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self.lock:
                raw = self.raw
                baseline = self.baseline
                relative = self.relative
                healthy = self.healthy
                status = self.status
                cal_status = self.cal_status
                samples = list(self.history)
                age = time.monotonic() - self.last_raw_at
                note = self.note if time.monotonic() < self.note_until else ''
            canvas = np.full((530, 760, 3), (29, 32, 37), dtype=np.uint8)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(canvas, 'MQ-6 / POP32i BASELINE CALIBRATION', (26, 40),
                        font, .8, (244, 244, 244), 2, cv2.LINE_AA)
            cv2.putText(canvas, 'ADC RAW: {}'.format(raw if raw is not None else '--'),
                        (30, 93), font, .72, (240, 240, 240), 2)
            cv2.putText(canvas, 'BASELINE: {}'.format(baseline if baseline is not None else '--'),
                        (300, 93), font, .72, (240, 240, 240), 2)
            cv2.putText(canvas, 'CHANGE: {}'.format(
                '{:+.1f}%'.format(relative) if relative is not None else '--'),
                (560, 93), font, .60, (240, 240, 240), 2)
            live = healthy and age < 3.0
            cv2.putText(canvas, 'SENSOR: {} / {}'.format('OK' if live else 'NO FRESH DATA', status[:42]),
                        (30, 128), font, .51, (95, 220, 130) if live else (100, 120, 245), 1)
            cv2.putText(canvas, 'CALIBRATION STATUS: {}'.format(cal_status[:48]),
                        (30, 154), font, .52, (210, 210, 210), 1)
            cv2.rectangle(canvas, (30, 178), (730, 380), (70, 73, 78), 1)
            if samples:
                # Fixed ADC 0..1023 axis so chart cannot conceal changes through autoscaling.
                positions = np.linspace(33, 726, len(samples))
                xy = np.array([(int(x), int(376 - 194*s/1023.0))
                               for x, s in zip(positions, samples)], np.int32)
                if len(xy) > 1:
                    cv2.polylines(canvas, [xy], False, (80, 205, 249), 2, cv2.LINE_AA)
                if baseline is not None:
                    y = int(376 - 194*baseline/1023.0)
                    cv2.line(canvas, (30, y), (730, y), (130, 228, 125), 1)
            cv2.putText(canvas, '0', (36, 375), font, .38, (176, 176, 176), 1)
            cv2.putText(canvas, '1023', (34, 192), font, .38, (176, 176, 176), 1)
            cv2.rectangle(canvas, (30, 405), (730, 463), (60, 111, 135), -1)
            cv2.putText(canvas, 'CALIBRATE CLEAN-AIR BASELINE  [C]', (105, 443),
                        font, .7, (250, 250, 250), 2, cv2.LINE_AA)
            if self.confirm:
                cv2.putText(canvas, 'Confirm CLEAN AIR + warmed sensor: Y=yes / N=cancel',
                            (33, 493), font, .48, (145, 205, 250), 1)
            else:
                cv2.putText(canvas, 'Reference only: raw ADC baseline, NOT ppm or %LEL.',
                            (33, 490), font, .47, (186, 187, 190), 1)
            if note:
                cv2.putText(canvas, note[:84], (33, 517), font, .42, (210, 212, 220), 1)
            cv2.imshow(self.window, canvas)
            key = cv2.waitKeyEx(1)
            if key >= 0:
                k = key & 0xff
                if self.confirm:
                    if k in (ord('y'), ord('Y')):
                        self.start_calibration()
                    elif k in (ord('n'), ord('N'), 27):
                        self.confirm = False
                elif k in (ord('c'), ord('C')):
                    self.confirm = True
                    self.notify('Confirm known clean air: press Y; press N to cancel.', 30)
                elif k in (ord('q'), ord('Q'), 27):
                    break
            rate.sleep()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('stip_mq6_baseline_gui')
    try:
        MQ6BaselineGUI().run()
    except Exception as exc:
        rospy.logerr('MQ6 baseline GUI failed: %s', exc)
        raise SystemExit(1)
