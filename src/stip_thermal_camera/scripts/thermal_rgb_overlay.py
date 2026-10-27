#!/usr/bin/env python3
"""Visual-only translucent registration of MLX90640 over RGB JPEG frames.

Subscribes to the EXISTING compressed inspection camera and 32FC1 thermal
measurements. Publishes /thermal/rgb_overlay; never modifies measurement topics.
"""
import os
import threading
import time

import cv2
import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage


class ThermalRgbOverlay:
    def __init__(self):
        self.bridge = CvBridge()
        self.rgb_topic = rospy.get_param('~rgb_topic', '/inspection_camera/image_raw/compressed')
        self.thermal_topic = rospy.get_param('~thermal_topic', '/thermal/image_raw')
        self.output_topic = rospy.get_param('~output_topic', '/thermal/rgb_overlay')
        self.align_path = os.path.expanduser(rospy.get_param('~alignment_file', ''))
        self.temperature_path = os.path.expanduser(rospy.get_param('~temperature_file', ''))
        self.alpha = float(rospy.get_param('~alpha', 0.40))
        self.fps = float(rospy.get_param('~display_fps', 20.0))
        self.blur = float(rospy.get_param('~blur_sigma', 2.0))
        self.min_c = float(rospy.get_param('~min_c', 15.0))
        self.max_c = float(rospy.get_param('~max_c', 45.0))
        self.auto_range = bool(rospy.get_param('~auto_range', True))
        self.min_span = max(2.0, float(rospy.get_param('~min_display_span_c', 8.0)))
        self.scale_low = None
        self.scale_high = None
        self.max_age = float(rospy.get_param('~max_thermal_age_sec', 0.6))
        self.show = bool(rospy.get_param('~show', True))
        self.decode_scale = int(rospy.get_param('~decode_scale', 2))
        if not (0.0 <= self.alpha <= 1.0 and 1.0 <= self.fps <= 60.0
                and self.max_c > self.min_c and self.max_age > 0
                and self.blur >= 0 and self.decode_scale in (1, 2, 4, 8)):
            raise ValueError('Invalid alpha/fps/temperature/age/blur/decode_scale parameters')
        if not os.path.isfile(self.align_path):
            raise RuntimeError('Alignment file missing: {}. Run align_thermal_to_rgb.py first.'.format(
                self.align_path))
        with open(self.align_path) as f:
            cfg = yaml.safe_load(f)
        self.H = np.asarray(cfg['H_thermal_to_rgb'], dtype=np.float64)
        self.ref_width = int(cfg['rgb_width'])
        self.ref_height = int(cfg['rgb_height'])
        if self.H.shape != (3, 3) or not np.isfinite(self.H).all() or abs(np.linalg.det(self.H)) < 1e-10:
            raise ValueError('Alignment homography is singular or invalid')
        if self.ref_width <= 0 or self.ref_height <= 0:
            raise ValueError('Invalid calibration image dimensions')
        self.gain, self.offset = 1.0, 0.0
        if self.temperature_path:
            if not os.path.isfile(self.temperature_path):
                raise RuntimeError('Temperature correction file not found: {}'.format(self.temperature_path))
            with open(self.temperature_path) as f:
                correction = yaml.safe_load(f)
            if correction.get('kind') != 'mlx90640_display_temperature_correction':
                raise ValueError('Not a valid temperature correction YAML')
            self.gain = float(correction['gain'])
            self.offset = float(correction['offset_c'])

        self.flags = {1: cv2.IMREAD_COLOR, 2: cv2.IMREAD_REDUCED_COLOR_2,
                      4: cv2.IMREAD_REDUCED_COLOR_4, 8: cv2.IMREAD_REDUCED_COLOR_8}
        self.lock = threading.Lock()
        self.latest_rgb_jpeg = None
        self.latest_rgb_stamp = rospy.Time(0)
        self.latest_t = None
        self.latest_t_stamp = rospy.Time(0)
        self.latest_t_received = 0.0
        self.rgb_cache = None
        self.rgb_cache_bytes = None
        self.cache_thermal_frame = None
        self.cache_warp = None
        self.cache_mask = None
        self.cache_h_size = None
        self.last_rgb_received = 0.0
        self.publisher = rospy.Publisher(self.output_topic, Image, queue_size=1)
        rospy.Subscriber(self.rgb_topic, CompressedImage, self.on_rgb,
                         queue_size=1, buff_size=4 * 1024 * 1024)
        rospy.Subscriber(self.thermal_topic, Image, self.on_thermal,
                         queue_size=1, buff_size=256 * 1024)
        if self.show:
            cv2.namedWindow('STIP THERMAL x RGB - Q/ESC closes overlay', cv2.WINDOW_NORMAL)
        rospy.loginfo('Thermal RGB overlay: %s + %s -> %s at %.0f display FPS',
                      self.thermal_topic, self.rgb_topic, self.output_topic, self.fps)

    def on_rgb(self, msg):
        with self.lock:
            self.latest_rgb_jpeg = bytes(msg.data)
            self.latest_rgb_stamp = msg.header.stamp
            self.last_rgb_received = time.monotonic()

    def on_thermal(self, msg):
        try:
            src = np.asarray(self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1'),
                             dtype=np.float32)
            if src.shape != (24, 32) or not np.isfinite(src).all():
                return
            if src.min() < -40 or src.max() > 300:
                return
            with self.lock:
                self.latest_t = src.copy()
                self.latest_t_stamp = msg.header.stamp
                self.latest_t_received = time.monotonic()
        except Exception as exc:
            rospy.logwarn_throttle(5, 'Overlay thermal image decode: %s', exc)

    def warped_heatmap(self, thermal, width, height):
        # Calibration H maps native (32x24) thermal pixels to RGB positions at
        # ref_width x ref_height. Scale it if output image resolution differs.
        scale = np.diag([width / self.ref_width, height / self.ref_height, 1.0])
        h = scale.dot(self.H)
        corrected = thermal * self.gain + self.offset
        valid = np.ones((24, 32), dtype=np.float32)
        warped_temp = cv2.warpPerspective(corrected, h, (width, height),
                                          flags=cv2.INTER_LINEAR)
        coverage = cv2.warpPerspective(valid, h, (width, height),
                                       flags=cv2.INTER_LINEAR)
        # Prevent 0-degree border bleed by normalizing the blur with coverage.
        if self.blur > 0:
            warped_temp = cv2.GaussianBlur(warped_temp, (0, 0), self.blur)
            coverage_smooth = cv2.GaussianBlur(coverage, (0, 0), self.blur)
        else:
            coverage_smooth = coverage
        normalized_temp = warped_temp / np.maximum(coverage_smooth, 1e-6)
        normalized_temp = np.clip(normalized_temp,
                                  float(corrected.min()), float(corrected.max()))
        # Auto-contrast from the unwarped 32x24 measurements only.
        # warped_heatmap is called on each newly received frame, not
        # every 20-FPS display tick, to avoid over-smoothing the range.
        if self.auto_range:
            lo, hi = np.percentile(corrected, [5, 95])
            middle = (float(lo) + float(hi)) / 2.0
            half = max((float(hi) - float(lo)) / 2.0, self.min_span / 2.0)
            next_low, next_high = middle - half, middle + half
            if self.scale_low is None:
                self.scale_low, self.scale_high = next_low, next_high
            else:
                self.scale_low = .85 * self.scale_low + .15 * next_low
                self.scale_high = .85 * self.scale_high + .15 * next_high
            low, high = self.scale_low, self.scale_high
        else:
            low, high = self.min_c, self.max_c
        gray = np.uint8(np.clip(
            (normalized_temp - low) * (255.0 / max(high - low, 0.01)),
            0.0, 255.0))
        heat = cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO)
        mask = coverage > 0.5
        return heat, mask

    def run(self):
        rate = rospy.Rate(self.fps)
        while not rospy.is_shutdown():
            with self.lock:
                jpeg = self.latest_rgb_jpeg
                rgb_stamp = self.latest_rgb_stamp
                tframe = self.latest_t
                t_stamp = self.latest_t_stamp
                age = time.monotonic() - self.latest_t_received
                rgb_age = time.monotonic() - self.last_rgb_received
            if jpeg is None or tframe is None:
                rate.sleep()
                continue
            if jpeg is not self.rgb_cache_bytes:
                decoded = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8),
                                       self.flags[self.decode_scale])
                if decoded is not None:
                    self.rgb_cache = decoded
                    self.rgb_cache_bytes = jpeg
            if self.rgb_cache is None:
                rate.sleep()
                continue
            rgb = self.rgb_cache.copy()
            height, width = rgb.shape[:2]
            fresh = age < self.max_age and rgb_age < 1.0
            if fresh:
                if tframe is not self.cache_thermal_frame or self.cache_h_size != (width, height):
                    self.cache_warp, self.cache_mask = self.warped_heatmap(tframe, width, height)
                    self.cache_thermal_frame = tframe
                    self.cache_h_size = (width, height)
                # Compose only the registered thermal FOV, leaving normal RGB
                # visible elsewhere. This is visual-only; raw Celsius unchanged.
                blended = cv2.addWeighted(rgb, 1.0 - self.alpha,
                                          self.cache_warp, self.alpha, 0)
                rgb[self.cache_mask] = blended[self.cache_mask]
                label = 'THERMAL + RGB | alpha %.2f | raw max %.1f C' % (
                    self.alpha, float(tframe.max()))
            else:
                label = 'THERMAL STALE / RGB STALE - OVERLAY DISABLED'
            cv2.rectangle(rgb, (0, 0), (min(width, 780), 34), (0, 0, 0), -1)
            cv2.putText(rgb, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                        .55, (255, 255, 255), 1, cv2.LINE_AA)
            try:
                ros_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='bgr8')
                ros_msg.header.stamp = rgb_stamp if rgb_stamp != rospy.Time(0) else rospy.Time.now()
                ros_msg.header.frame_id = 'inspection_camera_link'
                self.publisher.publish(ros_msg)
            except Exception as exc:
                rospy.logwarn_throttle(5, 'Overlay publish failed: %s', exc)
            if self.show:
                cv2.imshow('STIP THERMAL x RGB - Q/ESC closes overlay', rgb)
                if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
                    rospy.signal_shutdown('Overlay window closed')
                    break
            rate.sleep()
        if self.show:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('stip_thermal_rgb_overlay')
    try:
        ThermalRgbOverlay().run()
    except Exception as exc:
        rospy.logerr('Thermal RGB overlay failed: %s', exc)
        raise SystemExit(1)
