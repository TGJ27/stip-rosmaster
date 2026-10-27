#!/usr/bin/env python3
"""MLX90640 I2C (selected Linux bus) -> real 32x24 Celsius + smooth color preview.

Cubic interpolation is VISUAL only: maxima/minima are always measured from the
native 32x24 array. Never treat resampled pixels as additional measurements.
"""
import math
import os
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Float32MultiArray, String


REFRESH_MAP = {
    0.5: 'REFRESH_0_5_HZ',
    1.0: 'REFRESH_1_HZ',
    2.0: 'REFRESH_2_HZ',
    4.0: 'REFRESH_4_HZ',
    8.0: 'REFRESH_8_HZ',
    16.0: 'REFRESH_16_HZ',
    32.0: 'REFRESH_32_HZ',
    64.0: 'REFRESH_64_HZ',
}


class ThermalNode:
    def __init__(self):
        self.bus_num = int(rospy.get_param('~i2c_bus', 1))
        self.address = int(rospy.get_param('~address', 0x33))
        self.refresh = float(rospy.get_param('~refresh_hz', 2.0))
        self.out_w = max(64, int(rospy.get_param('~output_width', 320)))
        self.preview_blur_sigma = max(0.0, float(rospy.get_param('~preview_blur_sigma', 2.5)))
        self.out_h = max(48, int(rospy.get_param('~output_height', 240)))
        self.auto_range = bool(rospy.get_param('~auto_range', False))
        self.min_c = float(rospy.get_param('~min_c', 30.0))
        self.max_c = float(rospy.get_param('~max_c', 45.0))
        self.max_errors = max(1, int(rospy.get_param('~max_errors', 8)))
        self.frame_timeout = float(rospy.get_param('~frame_timeout_sec', 12.0))
        self.frame_id = rospy.get_param('~frame_id', 'mlx90640_optical')
        if self.max_c <= self.min_c:
            raise ValueError('max_c must be greater than min_c')
        if self.refresh not in REFRESH_MAP:
            raise ValueError('refresh_hz must be one of {}'.format(list(REFRESH_MAP)))

        self.bridge = CvBridge()
        self.raw_pub = rospy.Publisher('/thermal/image_raw', Image, queue_size=1)
        self.color_pub = rospy.Publisher('/thermal/image_color', Image, queue_size=1)
        self.array_pub = rospy.Publisher('/thermal/temperatures', Float32MultiArray, queue_size=1)
        self.minimum_pub = rospy.Publisher('/thermal/min_c', Float32, queue_size=1)
        self.maximum_pub = rospy.Publisher('/thermal/max_c', Float32, queue_size=1)
        self.center_pub = rospy.Publisher('/thermal/center_c', Float32, queue_size=1)
        self.healthy_pub = rospy.Publisher('/thermal/healthy', Bool, queue_size=1)
        self.status_pub = rospy.Publisher('/thermal/status', String, queue_size=1, latch=True)
        self.status_pub.publish(String(data='STARTING'))
        self.healthy_pub.publish(Bool(data=False))
        self.framebuf = [0.0] * 768
        self.scale_min = None
        self.scale_max = None
        self.mlx = None
        self.i2c = None
        rospy.on_shutdown(self.shutdown)

    def connect(self):
        device = '/dev/i2c-{}'.format(self.bus_num)
        if not os.path.exists(device):
            raise RuntimeError('{} not present. Check Jetson bus numbering.'.format(device))
        # Delayed imports make dependency failures clear in ROS logs.
        import adafruit_mlx90640
        from adafruit_extended_bus import ExtendedI2C
        # Jetson: 16 words (32 bytes) per I2C transaction.
        adafruit_mlx90640.I2C_READ_LEN = 128
        self.i2c = ExtendedI2C(self.bus_num)
        self.mlx = adafruit_mlx90640.MLX90640(self.i2c, address=self.address)
        self.mlx.refresh_rate = getattr(adafruit_mlx90640.RefreshRate,
                                         REFRESH_MAP[self.refresh])
        rospy.loginfo('MLX90640 on %s address 0x%02X, %.1f Hz',
                      device, self.address, self.refresh)
        self.status_pub.publish(String(data='CONNECTED_WAITING_FOR_FRAME'))

    def make_preview(self, measured):
        # All measurements (including hotspot) come from ORIGINAL 24x32 grid.
        if self.auto_range:
            low = float(np.percentile(measured, 5))
            high = float(np.percentile(measured, 95))
            mid = 0.5 * (low + high)
            width = max(high - low, 8.0)
            low, high = mid - width / 2, mid + width / 2
            if self.scale_min is None:
                self.scale_min, self.scale_max = low, high
            else:
                self.scale_min = 0.85 * self.scale_min + 0.15 * low
                self.scale_max = 0.85 * self.scale_max + 0.15 * high
            low, high = self.scale_min, self.scale_max
        else:
            low, high = self.min_c, self.max_c

        resized = cv2.resize(measured, (self.out_w, self.out_h), interpolation=cv2.INTER_LINEAR)
        if self.preview_blur_sigma > 0:
            resized = cv2.GaussianBlur(resized, (0, 0), self.preview_blur_sigma)
        # Clamp cubic overshoot to measured native-frame extrema (no fake hot pixel).
        resized = np.clip(resized, float(measured.min()), float(measured.max()))
        gray = np.uint8(np.clip((resized - low) / (high - low) * 255.0, 0, 255))
        color = cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO)
        max_r, max_c = np.unravel_index(int(np.argmax(measured)), measured.shape)
        px = round((max_c + .5) * self.out_w / 32)
        py = round((max_r + .5) * self.out_h / 24)
        cv2.drawMarker(color, (px, py), (255, 255, 255), cv2.MARKER_CROSS, 13, 1)
        cv2.putText(color, 'MIN {:.1f}C MAX {:.1f}C'.format(measured.min(), measured.max()),
                    (7, 18), cv2.FONT_HERSHEY_SIMPLEX, .47, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(color, 'MIN {:.1f}C MAX {:.1f}C'.format(measured.min(), measured.max()),
                    (7, 18), cv2.FONT_HERSHEY_SIMPLEX, .47, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(color, 'SCALE {:.0f}-{:.0f}C'.format(low, high),
                    (7, self.out_h - 9), cv2.FONT_HERSHEY_SIMPLEX, .46, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(color, 'SCALE {:.0f}-{:.0f}C'.format(low, high),
                    (7, self.out_h - 9), cv2.FONT_HERSHEY_SIMPLEX, .46, (255, 255, 255), 1, cv2.LINE_AA)
        return color

    def publish_frame(self):
        self.mlx.getFrame(self.framebuf)  # driver fills all 768 measured pixels
        frame = np.array(self.framebuf, dtype=np.float32).reshape((24, 32))
        if not np.all(np.isfinite(frame)) or frame.min() < -40 or frame.max() > 300:
            raise ValueError('Invalid MLX90640 temperatures')
        preview = self.make_preview(frame)
        stamp = rospy.Time.now()
        raw_image = self.bridge.cv2_to_imgmsg(frame, encoding='32FC1')
        raw_image.header.stamp = stamp
        raw_image.header.frame_id = self.frame_id
        color_image = self.bridge.cv2_to_imgmsg(preview, encoding='bgr8')
        color_image.header = raw_image.header
        self.raw_pub.publish(raw_image)
        self.color_pub.publish(color_image)
        self.array_pub.publish(Float32MultiArray(data=frame.reshape(-1).tolist()))
        self.minimum_pub.publish(Float32(data=float(frame.min())))
        self.maximum_pub.publish(Float32(data=float(frame.max())))
        self.center_pub.publish(Float32(data=float(frame[12, 16])))
        self.healthy_pub.publish(Bool(data=True))
        self.status_pub.publish(String(data='OK'))

    def run(self):
        try:
            self.connect()
            consecutive_errors = 0
            last_good = time.monotonic()
            # getFrame() itself waits for the sensor's next frame; no separate sleep
            while not rospy.is_shutdown():
                try:
                    self.publish_frame()
                    last_good = time.monotonic()
                    consecutive_errors = 0
                except (ValueError, RuntimeError, OSError) as exc:
                    consecutive_errors += 1
                    # A single bad subframe is common with MLX90640 and is NOT a
                    # reason for a full ROS process restart. The manager uses the
                    # timestamp of the most recent valid frame to detect staleness.
                    self.status_pub.publish(String(data='FRAME_RETRY: {}'.format(exc)[:170]))
                    rospy.logwarn_throttle(3, 'MLX90640 frame read failed: %s', exc)
                    if consecutive_errors >= self.max_errors or time.monotonic() - last_good > self.frame_timeout:
                        self.healthy_pub.publish(Bool(data=False))
                        raise RuntimeError('MLX90640 persistently failed; manager may retry')
                    rospy.sleep(0.15)
        finally:
            self._release_i2c()

    def shutdown(self):
        # ROSMASTER_LAUNCHER_V1_THERMAL_CLEANUP
        # ROS shutdown callbacks can run concurrently with getFrame().
        # Never close I2C here; the acquisition thread closes it in run.finally.
        try:
            self.healthy_pub.publish(Bool(data=False))
        except Exception:
            pass

    def _release_i2c(self):
        i2c = self.i2c
        self.i2c = None
        self.mlx = None
        if i2c is not None:
            try:
                i2c.deinit()
            except (AttributeError, OSError) as exc:
                # Some ExtendedI2C versions have incomplete teardown state.
                rospy.logdebug('I2C already released during exit: %s', exc)
            except Exception as exc:
                rospy.logwarn('I2C close warning: %s', exc)


if __name__ == '__main__':
    rospy.init_node('stip_thermal_camera')
    try:
        ThermalNode().run()
    except Exception as error:
        if rospy.is_shutdown():
            rospy.loginfo('MLX90640 acquisition ended during requested ROS shutdown: %s', error)
        else:
            rospy.logerr('MLX90640 node failed: %s', error)
            raise SystemExit(1)
