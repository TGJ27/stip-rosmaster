#!/usr/bin/env python3
"""MLX90640 smooth display: spatial upsampling + temporal transitions.

The original 32x24 /thermal/image_raw frames are the ONLY actual measurements.
30 FPS here refers to the preview refresh, not to the camera measurement rate.
Never publish blended temperatures as real sensor measurements.
"""
import threading
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Viewer:
    def __init__(self):
        self.bridge = CvBridge()
        self.window = 'STIP MLX90640 - Smooth Preview (Q / ESC to close)'
        self.display_fps = max(5.0, min(60.0, float(rospy.get_param('~display_fps', 30.0))))
        self.width = max(64, int(rospy.get_param('~width', 640)))
        self.height = max(48, int(rospy.get_param('~height', 480)))
        self.blur_sigma = max(0.0, float(rospy.get_param('~blur_sigma', 4.0)))
        self.auto_range = bool(rospy.get_param('~auto_range', True))
        self.min_span = max(2.0, float(rospy.get_param('~min_display_span_c', 8.0)))
        self.low_fixed = float(rospy.get_param('~min_c', 15.0))
        self.high_fixed = float(rospy.get_param('~max_c', 45.0))
        self.smoothing_fraction = max(0.1, min(1.0, float(rospy.get_param('~transition_fraction', 0.85))))
        self.max_blend_seconds = max(0.1, float(rospy.get_param('~max_blend_seconds', 0.6)))

        self.lock = threading.Lock()
        self.pending = None
        self.pending_time = None
        self.start = None
        self.target = None
        self.start_at = 0.0
        self.duration = 0.0
        self.last_rx_at = None
        self.last_rx_interval = None
        self.target_low = None
        self.target_high = None
        self.scale_low = None
        self.scale_high = None
        self.had_valid = False
        self.latest_native = None

        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window, self.width, self.height)
        rospy.Subscriber('/thermal/image_raw', Image, self.callback, queue_size=1, buff_size=1024 * 256)
        rospy.loginfo('Thermal smooth viewer: %.0f display FPS; real MLX90640 frames via /thermal/image_raw', self.display_fps)

    def callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            if image.shape != (24, 32):
                rospy.logwarn_throttle(5, 'Unexpected thermal frame size: %s', image.shape)
                return
            raw = np.asarray(image, dtype=np.float32).copy()
            if not np.all(np.isfinite(raw)) or raw.min() < -40 or raw.max() > 300:
                return
            with self.lock:
                self.pending = raw
                self.pending_time = time.monotonic()
        except Exception as exc:
            rospy.logwarn_throttle(5, 'Thermal preview decode failed: %s', exc)

    def blend_at(self, now):
        if self.target is None:
            return None
        if self.start is None or self.duration <= 0:
            return self.target
        progress = max(0.0, min(1.0, (now - self.start_at) / self.duration))
        progress = progress * progress * (3.0 - 2.0 * progress)
        return self.start * (1.0 - progress) + self.target * progress

    def apply_pending(self, now):
        with self.lock:
            incoming, received_at = self.pending, self.pending_time
            self.pending = None
        if incoming is None:
            return

        current = self.blend_at(now)
        if current is None:
            self.start = None
            self.target = incoming
            self.duration = 0.0
        else:
            self.start = current.copy()
            self.target = incoming
            if self.last_rx_at is None:
                interval = 0.25
            else:
                interval = max(0.05, received_at - self.last_rx_at)
            self.last_rx_interval = interval
            self.duration = min(self.max_blend_seconds, interval * self.smoothing_fraction)
        self.start_at = now
        self.last_rx_at = received_at
        self.latest_native = incoming
        self.had_valid = True

        if self.auto_range:
            low, high = np.percentile(incoming, [5, 95])
            center = (float(low) + float(high)) / 2.0
            half_span = max((float(high) - float(low)) / 2.0, self.min_span / 2.0)
            self.target_low, self.target_high = center - half_span, center + half_span
            if self.scale_low is None:
                self.scale_low = self.target_low
                self.scale_high = self.target_high
            else:
                # Update ranges only when actual sensor frames arrive.
                self.scale_low = 0.85 * self.scale_low + 0.15 * self.target_low
                self.scale_high = 0.85 * self.scale_high + 0.15 * self.target_high
        else:
            self.scale_low, self.scale_high = self.low_fixed, self.high_fixed

    def render(self, current, now):
        # Interpolated image is DISPLAY ONLY, not a new measurement.
        upscaled = cv2.resize(current, (self.width, self.height), interpolation=cv2.INTER_CUBIC)
        if self.blur_sigma > 0:
            upscaled = cv2.GaussianBlur(upscaled, (0, 0), self.blur_sigma)
        # Prevent cubic interpolation overshoots beyond measured pixel extrema.
        lo_native = float(min(self.start.min(), self.target.min())) if self.start is not None else float(self.target.min())
        hi_native = float(max(self.start.max(), self.target.max())) if self.start is not None else float(self.target.max())
        upscaled = np.clip(upscaled, lo_native, hi_native)
        denom = max(0.01, self.scale_high - self.scale_low)
        gray = np.uint8(np.clip((upscaled - self.scale_low) / denom * 255.0, 0, 255))
        color = cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO)

        latest = self.latest_native
        hotspot_row, hotspot_col = np.unravel_index(np.argmax(latest), latest.shape)
        x = int((hotspot_col + .5) * self.width / 32)
        y = int((hotspot_row + .5) * self.height / 24)
        cv2.drawMarker(color, (x, y), (255, 255, 255), cv2.MARKER_CROSS, 18, 1)

        measurement_hz = 0.0 if not self.last_rx_interval else 1.0 / self.last_rx_interval
        freshness = now - self.last_rx_at
        status = 'LIVE' if freshness < 3.0 else 'STALE (no new thermal frame)'
        font_scale = max(.5, min(1., self.width / 650.0))
        lines = [
            'MIN %.1fC  MAX %.1fC  CENTER %.1fC' % (latest.min(), latest.max(), latest[12, 16]),
            'Actual sensor: %.1f Hz   Display: %.0f FPS   %s' % (measurement_hz, self.display_fps, status),
            'Display range: %.1f - %.1fC' % (self.scale_low, self.scale_high),
        ]
        for i, line in enumerate(lines):
            origin = (9, 24 + i * int(25 * font_scale + 4))
            cv2.putText(color, line, origin, cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(color, line, origin, cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), 1, cv2.LINE_AA)
        return color

    def run(self):
        rate = rospy.Rate(self.display_fps)
        while not rospy.is_shutdown():
            now = time.monotonic()
            self.apply_pending(now)
            if self.had_valid:
                current = self.blend_at(now)
                if current is not None:
                    cv2.imshow(self.window, self.render(current, now))
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                rospy.signal_shutdown('Thermal viewer closed')
                break
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('stip_thermal_viewer')
    Viewer().run()
