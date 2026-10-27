#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os
import re
import threading
import time
from datetime import datetime

import cv2
import numpy as np
import rospy

try:
    from pyzbar import pyzbar
    PYZBAR_AVAILABLE = True
except ImportError:
    pyzbar = None
    PYZBAR_AVAILABLE = False

from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64MultiArray
from stip_rosmaster_msgs.msg import ArmJoint
from stip_rosmaster_msgs.srv import RobotArmArray

# Resolve the Python helper from the catkin package source tree; works with
# generated ROS Noetic devel-space script wrappers as well.
import importlib.util as _importlib_util
import os as _decoder_os
import rospkg as _decoder_rospkg
_decoder_file = _decoder_os.path.join(
    _decoder_rospkg.RosPack().get_path('stip_qr_inspection'),
    'scripts', 'stip_hw_jpeg.py')
_decoder_spec = _importlib_util.spec_from_file_location('stip_hw_jpeg', _decoder_file)
_decoder_module = _importlib_util.module_from_spec(_decoder_spec)
_decoder_spec.loader.exec_module(_decoder_module)
JpegDecoder = _decoder_module.JpegDecoder

from stip_qr_inspection.srv import (
    EnrollInspection,
    EnrollInspectionResponse,
    RunInspection,
    RunInspectionResponse,
)


class QRInspectionNode:
    def __init__(self):
        # Stop OpenCV from spawning a large worker pool.
        self.opencv_threads = int(
            rospy.get_param("~opencv_threads", 2)
        )
        cv2.setNumThreads(
            max(1, self.opencv_threads)
        )

        self.decoder_backend = rospy.get_param('~decoder_backend', 'nvidia')
        # This decoder is lazy: no GStreamer processing until I/E starts QR.
        self.jpeg_decoder = JpegDecoder(
            self.decoder_backend, reduced=False, label='QR backend')
        rospy.loginfo('QR backend JPEG decoder requested: %s',
                      self.decoder_backend)

        self.image_topic = rospy.get_param(
            "~image_topic",
            "/inspection_camera/image_raw/compressed"
        )

        self.geometry_topic = rospy.get_param(
            "~geometry_topic",
            "/inspection/geometry"
        )

        self.data_dir = os.path.expanduser(
            rospy.get_param(
                "~data_dir",
                "~/Documents/stip_inspection_data"
            )
        )

        # QR only runs on E/I.
        self.qr_search_timeout = float(
            rospy.get_param(
                "~qr_search_timeout",
                5.0
            )
        )

        # Pan-only QR acquisition search used by Inspect.
        self.pan_search_enabled = bool(
            rospy.get_param("~pan_search_enabled", True)
        )
        self.pan_search_step_deg = float(
            rospy.get_param("~pan_search_step_deg", 1.0)
        )
        self.pan_search_interval = float(
            rospy.get_param("~pan_search_interval", 0.18)
        )
        self.pan_search_probe_timeout = float(
            rospy.get_param("~pan_search_probe_timeout", 0.20)
        )

        # Tracking loops.
        self.tracking_detection_hz = float(
            rospy.get_param(
                "~tracking_detection_hz",
                10.0
            )
        )
        self.tracking_control_hz = float(
            rospy.get_param(
                "~tracking_control_hz",
                10.0
            )
        )
        self.tracking_servo_run_time = int(
            rospy.get_param(
                "~tracking_servo_run_time",
                100
            )
        )
        self.alignment_timeout = float(
            rospy.get_param(
                "~alignment_timeout",
                20.0
            )
        )

        # Alignment thresholds are in FULL 1920x1080 camera pixels.
        self.tolerance_px = float(
            rospy.get_param(
                "~tolerance_px",
                20.0
            )
        )
        self.margin_px = float(
            rospy.get_param(
                "~margin_px",
                30.0
            )
        )

        # No exact centering requirement. Success means BOTH the QR and
        # indicator are comfortably visible inside the image.
        # Normal success condition:
        # combined QR + indicator center must be within this many pixels
        # from the image center, in both X and Y.
        self.center_range_px = float(
            rospy.get_param(
                "~center_range_px",
                200.0
            )
        )

        # Fallback only when a servo physically reaches its limit:
        # QR + indicator must remain at least this far inside all image edges.
        self.edge_margin_px = float(
            rospy.get_param(
                "~edge_margin_px",
                20.0
            )
        )

        self.track_far_px = float(
            rospy.get_param(
                "~track_far_px",
                300.0
            )
        )
        self.track_mid_px = float(
            rospy.get_param(
                "~track_mid_px",
                120.0
            )
        )
        self.track_far_step = float(
            rospy.get_param(
                "~track_far_step",
                3.0
            )
        )
        self.track_mid_step = float(
            rospy.get_param(
                "~track_mid_step",
                2.0
            )
        )
        self.track_near_step = float(
            rospy.get_param(
                "~track_near_step",
                1.0
            )
        )

        # Enrollment ROI window.
        self.roi_preview_max_width = int(
            rospy.get_param(
                "~roi_preview_max_width",
                960
            )
        )
        self.roi_preview_max_height = int(
            rospy.get_param(
                "~roi_preview_max_height",
                540
            )
        )

        # Arm.
        self.pan_servo_id = int(
            rospy.get_param("~pan_servo_id", 1)
        )
        self.tilt_servo_id = int(
            rospy.get_param("~tilt_servo_id", 2)
        )

        self.pan_home = float(
            rospy.get_param("~pan_home", 90.0)
        )
        self.tilt_home = float(
            rospy.get_param("~tilt_home", 0.0)
        )

        self.pan_min = float(
            rospy.get_param("~pan_min", 0.0)
        )
        self.pan_max = float(
            rospy.get_param("~pan_max", 180.0)
        )
        self.tilt_min = float(
            rospy.get_param("~tilt_min", 0.0)
        )
        self.tilt_max = float(
            rospy.get_param("~tilt_max", 90.0)
        )

        self.servo_run_time = int(
            rospy.get_param("~servo_run_time", 150)
        )

        # Latest COMPRESSED camera frame only.
        self.jpeg_lock = threading.Lock()
        self.latest_jpeg = None
        self.latest_stamp = rospy.Time(0)
        self.jpeg_seq = 0

        self.pose_lock = threading.Lock()
        self.pan = self.pan_home
        self.tilt = self.tilt_home

        self.qr = cv2.QRCodeDetector()

        self.profiles_dir = os.path.join(
            self.data_dir,
            "profiles"
        )
        self.runs_dir = os.path.join(
            self.data_dir,
            "runs"
        )
        os.makedirs(self.profiles_dir, exist_ok=True)
        os.makedirs(self.runs_dir, exist_ok=True)

        # Arm publisher is SESSION-SCOPED to QR enrollment/inspection. An idle
        # /TargetAngle publisher prevents the fire inspection controller from
        # obtaining exclusive access to the arm while both nodes are running.
        self.arm_pub = None
        self.arm_pub_lock = threading.RLock()
        self.arm_session_lock = threading.Lock()

        # Tiny geometry message used by the UI to draw boxes on its own
        # continuously updating camera preview. This avoids switching the UI
        # to a separately encoded inspection video stream.
        self.geometry_pub = rospy.Publisher(
            self.geometry_topic,
            Float64MultiArray,
            queue_size=1
        )

        rospy.Subscriber(
            self.image_topic,
            CompressedImage,
            self.jpeg_callback,
            queue_size=1,
            buff_size=4 * 1024 * 1024,
            tcp_nodelay=True
        )

        rospy.Subscriber(
            "/ArmAngleUpdate",
            ArmJoint,
            self.arm_update_callback,
            queue_size=20,
            tcp_nodelay=True
        )

        rospy.Service(
            "/inspection/enroll",
            EnrollInspection,
            self.enroll_callback
        )

        rospy.Service(
            "/inspection/run",
            RunInspection,
            self.run_callback
        )

        rospy.Service(
            "/inspection/warmup", Trigger,
            self.warmup_callback
        )

        self.try_read_arm_position()

        rospy.loginfo(
            "STIP inspection backend ready."
        )
        rospy.loginfo(
            "Idle mode keeps JPEG compressed; QR decode = OFF."
        )
        rospy.loginfo(
            "Tracking: detector %.1f Hz / controller %.1f Hz",
            self.tracking_detection_hz,
            self.tracking_control_hz
        )
        rospy.loginfo(
            "OpenCV threads limited to %d",
            self.opencv_threads
        )

    # --------------------------------------------------------------
    # Camera bytes
    # --------------------------------------------------------------

    def jpeg_callback(self, msg):
        # This is the only camera work while idle: copy compressed JPEG bytes.
        with self.jpeg_lock:
            # Keep the newest compressed JPEG and its ORIGINAL camera timestamp.
            self.latest_jpeg = msg.data
            self.latest_stamp = msg.header.stamp
            self.jpeg_seq += 1

    def get_jpeg_packet(self):
        with self.jpeg_lock:
            if self.latest_jpeg is None:
                return None, self.jpeg_seq

            return (
                self.latest_jpeg,
                int(self.jpeg_seq),
                self.latest_stamp
            )

    def decode_jpeg(self, jpeg_bytes, reduced=False):
        return self.jpeg_decoder.decode(jpeg_bytes)

    def warmup_callback(self, _req):
        """Decode and probe ONE live frame; never command the arm or enroll."""
        packet = self.get_jpeg_packet()
        if not packet or packet[0] is None:
            return TriggerResponse(success=False, message="No camera frame yet")
        try:
            frame = self.decode_jpeg(packet[0], reduced=False)
            if frame is None or frame.size == 0:
                return TriggerResponse(success=False, message="QR JPEG decode returned no pixels")
            # QR presence is NOT required to pass warmup; probe the detector.
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
            self.qr.detect(gray)
            return TriggerResponse(success=True, message="One frame decoded; QR detector initialized")
        except Exception as exc:
            rospy.logwarn("QR one-frame warmup failed: %s", exc)
            return TriggerResponse(success=False, message=str(exc))

    @staticmethod
    def resize_to_fit(
        frame,
        max_width,
        max_height
    ):
        h, w = frame.shape[:2]

        scale = min(
            1.0,
            float(max_width) / float(w),
            float(max_height) / float(h)
        )

        if scale >= 1.0:
            return frame.copy(), 1.0

        new_w = max(
            1,
            int(round(w * scale))
        )
        new_h = max(
            1,
            int(round(h * scale))
        )

        return (
            cv2.resize(
                frame,
                (new_w, new_h),
                interpolation=cv2.INTER_AREA
            ),
            scale
        )

    # --------------------------------------------------------------
    # Arm
    # --------------------------------------------------------------

    def arm_update_callback(self, msg):
        with self.pose_lock:
            if msg.id == self.pan_servo_id:
                self.pan = float(msg.angle)
            elif msg.id == self.tilt_servo_id:
                self.tilt = float(msg.angle)

    def try_read_arm_position(self):
        try:
            rospy.wait_for_service(
                "/CurrentAngle",
                timeout=1.0
            )

            proxy = rospy.ServiceProxy(
                "/CurrentAngle",
                RobotArmArray
            )

            response = proxy("get")
            angles = list(response.angles)

            with self.pose_lock:
                pan_index = self.pan_servo_id - 1
                tilt_index = self.tilt_servo_id - 1

                if (
                    pan_index < len(angles)
                    and angles[pan_index] >= 0
                ):
                    self.pan = float(
                        angles[pan_index]
                    )

                if (
                    tilt_index < len(angles)
                    and angles[tilt_index] >= 0
                ):
                    self.tilt = float(
                        angles[tilt_index]
                    )

        except Exception:
            pass

    def get_pose(self):
        with self.pose_lock:
            return float(self.pan), float(self.tilt)

    @staticmethod
    def clamp(
        value,
        low,
        high
    ):
        return max(
            low,
            min(high, value)
        )

    def send_servo(
        self,
        servo_id,
        angle,
        run_time=None
    ):
        if run_time is None:
            run_time = self.servo_run_time

        msg = ArmJoint()
        msg.id = int(servo_id)
        msg.run_time = int(run_time)
        msg.angle = float(angle)
        msg.joints = []

        with self.arm_pub_lock:
            if self.arm_pub is None:
                self.arm_pub = rospy.Publisher('/TargetAngle', ArmJoint, queue_size=10)
                # rospy needs a brief publisher/subscriber handshake before the
                # first servo target; otherwise a one-shot move may be lost.
                rospy.sleep(0.16)
            self.arm_pub.publish(msg)

        with self.pose_lock:
            if servo_id == self.pan_servo_id:
                self.pan = float(angle)
            elif servo_id == self.tilt_servo_id:
                self.tilt = float(angle)

    def move_to(
        self,
        pan,
        tilt,
        wait=True
    ):
        pan = self.clamp(
            float(pan),
            self.pan_min,
            self.pan_max
        )
        tilt = self.clamp(
            float(tilt),
            self.tilt_min,
            self.tilt_max
        )

        self.send_servo(
            self.pan_servo_id,
            pan
        )

        rospy.sleep(0.03)

        self.send_servo(
            self.tilt_servo_id,
            tilt
        )

        if wait:
            rospy.sleep(
                self.servo_run_time
                / 1000.0
                + 0.08
            )

    def tracking_step(self, error_px):
        error_px = abs(
            float(error_px)
        )

        if error_px > self.track_far_px:
            return self.track_far_step

        if error_px > self.track_mid_px:
            return self.track_mid_step

        if error_px > self.tolerance_px:
            return self.track_near_step

        return 0.0

    @staticmethod
    def combined_bounds(qr_points, indicator_points):
        points = np.vstack([
            np.asarray(qr_points, dtype=np.float32).reshape(-1, 2),
            np.asarray(indicator_points, dtype=np.float32).reshape(-1, 2),
        ])

        return (
            float(points[:, 0].min()),
            float(points[:, 1].min()),
            float(points[:, 0].max()),
            float(points[:, 1].max()),
        )

    def visibility_state(self, frame, qr_points, indicator_points):
        """
        Two separate success conditions.

        NORMAL:
            combined QR + indicator center must be within center_range_px
            of image center.

        SERVO-LIMIT FALLBACK:
            if motion is impossible because a servo is already at a limit,
            accept as long as QR + indicator remain inside edge_margin_px.
        """
        h, w = frame.shape[:2]

        min_x, min_y, max_x, max_y = self.combined_bounds(
            qr_points,
            indicator_points
        )

        target_center_x = (min_x + max_x) * 0.5
        target_center_y = (min_y + max_y) * 0.5

        image_center_x = float(w) * 0.5
        image_center_y = float(h) * 0.5

        error_x = target_center_x - image_center_x
        error_y = target_center_y - image_center_y

        center_range = max(
            0.0,
            float(self.center_range_px)
        )

        center_ok = (
            abs(error_x) <= center_range
            and abs(error_y) <= center_range
        )

        edge_margin = max(
            0.0,
            float(self.edge_margin_px)
        )

        edge_ok = (
            min_x >= edge_margin
            and min_y >= edge_margin
            and max_x <= float(w) - edge_margin
            and max_y <= float(h) - edge_margin
        )

        return {
            "error_x": float(error_x),
            "error_y": float(error_y),
            "center_ok": bool(center_ok),
            "edge_ok": bool(edge_ok),
        }

    # --------------------------------------------------------------
    # QR
    # --------------------------------------------------------------

    @staticmethod
    def order_quad_points(points):
        points = np.asarray(
            points,
            dtype=np.float32
        ).reshape(-1, 2)

        if points.shape[0] != 4:
            return None

        ordered = np.zeros(
            (4, 2),
            dtype=np.float32
        )

        sums = points.sum(axis=1)
        diffs = np.diff(
            points,
            axis=1
        ).reshape(-1)

        ordered[0] = points[np.argmin(sums)]
        ordered[2] = points[np.argmax(sums)]
        ordered[1] = points[np.argmin(diffs)]
        ordered[3] = points[np.argmax(diffs)]

        return ordered

    @staticmethod
    def bbox_corners(
        x,
        y,
        w,
        h
    ):
        return np.array(
            [
                [x, y],
                [x + w, y],
                [x + w, y + h],
                [x, y + h],
            ],
            dtype=np.float32
        )

    def detect_qr_zbar(
        self,
        frame,
        expected=None
    ):
        if not PYZBAR_AVAILABLE:
            return None, None

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        try:
            barcodes = pyzbar.decode(
                gray
            )
        except Exception:
            return None, None

        candidates = []

        for barcode in barcodes:
            if getattr(
                barcode,
                "type",
                ""
            ) != "QRCODE":
                continue

            try:
                value = barcode.data.decode(
                    "utf-8"
                ).strip()
            except Exception:
                value = barcode.data.decode(
                    "utf-8",
                    errors="replace"
                ).strip()

            if not value:
                continue

            polygon = getattr(
                barcode,
                "polygon",
                None
            ) or []

            points = np.array(
                [
                    [p.x, p.y]
                    for p in polygon
                ],
                dtype=np.float32
            )

            if points.shape[0] != 4:
                rect = barcode.rect

                points = self.bbox_corners(
                    rect.left,
                    rect.top,
                    rect.width,
                    rect.height
                )

            points = self.order_quad_points(
                points
            )

            if points is None:
                continue

            candidates.append(
                (value, points)
            )

        if expected is not None:
            for value, points in candidates:
                if value == expected:
                    return value, points

            return None, None

        if candidates:
            return candidates[0]

        return None, None

    def detect_qr_geometry(
        self,
        frame,
        previous_points=None,
        expected=None
    ):
        """
        Tracking path:
        1. Search around previous QR position using OpenCV geometry-only detect.
        2. If geometry detect fails, pyzbar-decode the SMALL crop.
        3. Caller can occasionally do full-frame pyzbar reacquisition.

        This avoids decoding the entire QR payload on every tracking frame.
        """

        h, w = frame.shape[:2]

        if previous_points is None:
            try:
                ok, points = self.qr.detect(frame)
            except Exception:
                ok, points = False, None

            if ok and points is not None:
                points = self.order_quad_points(
                    points
                )

                if points is not None:
                    return points

            return None

        previous_points = np.asarray(
            previous_points,
            dtype=np.float32
        ).reshape(-1, 2)

        min_x = float(
            previous_points[:, 0].min()
        )
        max_x = float(
            previous_points[:, 0].max()
        )
        min_y = float(
            previous_points[:, 1].min()
        )
        max_y = float(
            previous_points[:, 1].max()
        )

        qr_w = max(
            40.0,
            max_x - min_x
        )
        qr_h = max(
            40.0,
            max_y - min_y
        )

        # Wide crop so a few degrees of servo motion does not lose the QR.
        margin_x = qr_w * 1.75
        margin_y = qr_h * 1.75

        x1 = max(
            0,
            int(min_x - margin_x)
        )
        y1 = max(
            0,
            int(min_y - margin_y)
        )
        x2 = min(
            w,
            int(max_x + margin_x)
        )
        y2 = min(
            h,
            int(max_y + margin_y)
        )

        if x2 <= x1 or y2 <= y1:
            return None

        crop = frame[
            y1:y2,
            x1:x2
        ]

        # Cheap geometry-only detector first.
        try:
            ok, points = self.qr.detect(
                crop
            )
        except Exception:
            ok, points = False, None

        if ok and points is not None:
            points = self.order_quad_points(
                points
            )

            if points is not None:
                points[:, 0] += x1
                points[:, 1] += y1

                return points

        # Fallback: decode only the small crop.
        _, points = self.detect_qr_zbar(
            crop,
            expected
        )

        if points is not None:
            points[:, 0] += x1
            points[:, 1] += y1

            return points

        return None

    def wait_for_expected_qr(
        self,
        expected=None,
        timeout=None,
        min_seq=None
    ):
        if timeout is None:
            timeout = self.qr_search_timeout

        deadline = time.monotonic() + timeout
        last_seq = -1

        while (
            not rospy.is_shutdown()
            and time.monotonic() < deadline
        ):
            jpeg_bytes, seq, stamp = self.get_jpeg_packet()

            if jpeg_bytes is None:
                rospy.sleep(0.02)
                continue

            if (
                min_seq is not None
                and seq <= min_seq
            ):
                rospy.sleep(0.01)
                continue

            if seq == last_seq:
                rospy.sleep(0.01)
                continue

            last_seq = seq

            frame = self.decode_jpeg(
                jpeg_bytes,
                reduced=False
            )

            if frame is None:
                continue

            value, points = self.detect_qr_zbar(
                frame,
                expected
            )

            if points is not None:
                return {
                    "jpeg": jpeg_bytes,
                    "seq": seq,
                    "stamp": stamp,
                    "frame": frame,
                    "value": value,
                    "qr": points,
                }

        return None

    # --------------------------------------------------------------
    # Indicator geometry
    # --------------------------------------------------------------

    def project_indicator(
        self,
        profile,
        current_qr
    ):
        reference_qr = np.asarray(
            profile["qr"]["corners"],
            dtype=np.float32
        )

        reference_indicator = np.asarray(
            profile["indicator"]["corners"],
            dtype=np.float32
        )

        transform = cv2.getPerspectiveTransform(
            reference_qr,
            current_qr
        )

        return cv2.perspectiveTransform(
            reference_indicator.reshape(
                1,
                -1,
                2
            ),
            transform
        )[0]

    def make_measurement(
        self,
        packet,
        qr_points,
        indicator_points
    ):
        frame = packet["frame"]

        h, w = frame.shape[:2]

        indicator_center = np.mean(
            indicator_points,
            axis=0
        )

        image_center = np.array(
            [w / 2.0, h / 2.0],
            dtype=np.float32
        )

        fit_ok = all(
            self.margin_px
            <= point[0]
            <= w - self.margin_px
            and
            self.margin_px
            <= point[1]
            <= h - self.margin_px
            for point in indicator_points
        )

        return {
            "jpeg": packet["jpeg"],
            "seq": packet["seq"],
            "stamp": packet.get("stamp", rospy.Time.now()),
            "frame": frame,
            "qr": qr_points,
            "indicator": indicator_points,
            "indicator_center": indicator_center,
            "image_center": image_center,
            "fit_ok": fit_ok,
            "time": time.monotonic(),
        }

    # --------------------------------------------------------------
    # Overlay
    # --------------------------------------------------------------

    def annotate(
        self,
        frame,
        name,
        qr_value,
        qr_points,
        indicator_points,
        aligned=False
    ):
        out = frame.copy()

        if qr_points is not None:
            cv2.polylines(
                out,
                [
                    np.round(qr_points)
                    .astype(np.int32)
                    .reshape(-1, 1, 2)
                ],
                True,
                (0, 255, 0),
                3
            )

        if indicator_points is not None:
            cv2.polylines(
                out,
                [
                    np.round(indicator_points)
                    .astype(np.int32)
                    .reshape(-1, 1, 2)
                ],
                True,
                (255, 255, 0),
                3
            )

            center = np.mean(
                indicator_points,
                axis=0
            )

            cv2.circle(
                out,
                (
                    int(round(center[0])),
                    int(round(center[1]))
                ),
                8,
                (255, 255, 0),
                -1
            )

            label_point = tuple(
                np.round(
                    indicator_points[0]
                ).astype(int)
            )

            cv2.putText(
                out,
                "INDICATOR",
                (
                    max(5, label_point[0]),
                    max(25, label_point[1] - 8)
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 0),
                2,
                cv2.LINE_AA
            )

        h, w = out.shape[:2]

        cv2.drawMarker(
            out,
            (
                w // 2,
                h // 2
            ),
            (255, 255, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=30,
            thickness=2
        )

        pan, tilt = self.get_pose()

        lines = [
            "Inspection: {}".format(name),
            "QR: {}".format(qr_value),
            "Pan: {:.1f} Tilt: {:.1f}".format(
                pan,
                tilt
            ),
            "Aligned: {}".format(
                "YES" if aligned else "NO"
            ),
        ]

        y = 30

        for line in lines:
            cv2.putText(
                out,
                line,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA
            )

            y += 28

        return out

    def publish_geometry(
        self,
        measurement,
        aligned=False
    ):
        """
        Publish timestamped geometry without a custom ROS message.

        Float64 layout:
          [0]  camera frame timestamp, seconds
          [1]  source width
          [2]  source height
          [3]  aligned (1.0 / 0.0)
          [4:12]   QR x/y coordinates, 4 corners
          [12:20]  indicator x/y coordinates, 4 corners

        Float64 is used so the frame timestamp keeps enough precision for
        matching against the UI's camera-frame history.
        """
        frame = measurement["frame"]
        h, w = frame.shape[:2]

        qr = np.asarray(
            measurement["qr"],
            dtype=np.float64
        ).reshape(4, 2)

        indicator = np.asarray(
            measurement["indicator"],
            dtype=np.float64
        ).reshape(4, 2)

        stamp = measurement.get(
            "stamp",
            rospy.Time.now()
        ).to_sec()

        msg = Float64MultiArray()
        msg.data = [
            float(stamp),
            float(w),
            float(h),
            1.0 if aligned else 0.0,
        ]
        msg.data.extend(
            qr.reshape(-1).tolist()
        )
        msg.data.extend(
            indicator.reshape(-1).tolist()
        )

        self.geometry_pub.publish(msg)

    def reference_pan_search_direction(self, profile):
        """
        Decide the search direction from enrollment geometry.

        Saved QR on right -> +1 -> same direction as D / pan+
        Saved QR on left  -> -1 -> same direction as A / pan-

        If QR was nearly centered, use the indicator as a hint:
        indicator left -> QR-side search right, and vice versa.
        """
        camera_width = float(
            profile.get("camera", {}).get("width", 1920)
        )
        center_x = camera_width * 0.5

        qr_points = np.asarray(
            profile["qr"]["corners"],
            dtype=np.float32
        ).reshape(-1, 2)

        qr_center_x = float(qr_points[:, 0].mean())
        delta = qr_center_x - center_x

        if abs(delta) >= 20.0:
            return 1.0 if delta > 0.0 else -1.0

        indicator_points = np.asarray(
            profile["indicator"]["corners"],
            dtype=np.float32
        ).reshape(-1, 2)

        indicator_center_x = float(
            indicator_points[:, 0].mean()
        )

        if indicator_center_x < center_x:
            return 1.0
        if indicator_center_x > center_x:
            return -1.0

        return 1.0

    def horizontal_qr_search(self, profile, expected_qr):
        """
        Inspect acquisition:
        - quick check at saved pose
        - if missing, slowly pan in saved QR direction
        - check after every pan step
        - tilt is NEVER changed
        - when pan limit is reached, only then start the final 5 s timer
        """
        _, start_seq, _ = self.get_jpeg_packet()

        found = self.wait_for_expected_qr(
            expected=expected_qr,
            timeout=self.pan_search_probe_timeout,
            min_seq=max(0, start_seq - 1)
        )

        if found is not None:
            rospy.loginfo(
                "QR found at stored pose; pan search not needed."
            )
            return found

        if not self.pan_search_enabled:
            return self.wait_for_expected_qr(
                expected=expected_qr,
                timeout=self.qr_search_timeout
            )

        direction = self.reference_pan_search_direction(
            profile
        )

        rospy.loginfo(
            "QR missing. Horizontal search %s. Tilt unchanged.",
            "RIGHT / pan+" if direction > 0 else "LEFT / pan-"
        )

        while not rospy.is_shutdown():
            pan, _ = self.get_pose()

            next_pan = self.clamp(
                pan
                + direction
                * abs(self.pan_search_step_deg),
                self.pan_min,
                self.pan_max
            )

            if abs(next_pan - pan) < 1e-6:
                rospy.loginfo(
                    "Pan limit %.1f deg reached. Starting final %.1f s QR timer.",
                    pan,
                    self.qr_search_timeout
                )

                return self.wait_for_expected_qr(
                    expected=expected_qr,
                    timeout=self.qr_search_timeout
                )

            self.send_servo(
                self.pan_servo_id,
                next_pan,
                run_time=self.tracking_servo_run_time
            )

            rospy.sleep(
                max(0.01, self.pan_search_interval)
            )

            _, probe_seq, _ = self.get_jpeg_packet()

            found = self.wait_for_expected_qr(
                expected=expected_qr,
                timeout=self.pan_search_probe_timeout,
                min_seq=max(0, probe_seq - 1)
            )

            if found is not None:
                rospy.loginfo(
                    "QR acquired during pan search at %.1f deg.",
                    next_pan
                )
                return found

        return None

    # --------------------------------------------------------------
    # Profiles
    # --------------------------------------------------------------

    @staticmethod
    def safe_name(name):
        value = re.sub(
            r"[^A-Za-z0-9_.-]+",
            "_",
            name.strip()
        ).strip("_")

        return value or "inspection"

    def load_profile(self, name):
        path = os.path.join(
            self.profiles_dir,
            self.safe_name(name),
            "profile.json"
        )

        if not os.path.isfile(path):
            return None, path

        with open(path, "r") as file:
            return json.load(file), path

    # --------------------------------------------------------------
    # ENROLL
    # --------------------------------------------------------------

    def _release_arm_session(self):
        with self.arm_pub_lock:
            pub = self.arm_pub
            self.arm_pub = None
            if pub is not None:
                pub.unregister()

    def enroll_callback(self, req):
        with self.arm_session_lock:
            try:
                return self._enroll_callback(req)
            finally:
                self._release_arm_session()

    def _enroll_callback(self, req):
        name = req.name.strip()

        if not name:
            return EnrollInspectionResponse(
                False,
                "Inspection name cannot be empty.",
                "",
                "",
                0.0,
                0.0
            )

        rospy.loginfo(
            "ENROLL: QR decode ON for max %.1f sec",
            self.qr_search_timeout
        )

        found = self.wait_for_expected_qr(
            expected=None,
            timeout=self.qr_search_timeout
        )

        if found is None:
            pan, tilt = self.get_pose()

            return EnrollInspectionResponse(
                False,
                "QR not found within {:.1f} seconds.".format(
                    self.qr_search_timeout
                ),
                "",
                "",
                pan,
                tilt
            )

        frame = found["frame"]
        jpeg_bytes = found["jpeg"]
        qr_value = found["value"]
        qr_points = found["qr"]

        pan, tilt = self.get_pose()

        full_h, full_w = frame.shape[:2]

        preview = frame.copy()

        cv2.polylines(
            preview,
            [
                np.round(qr_points)
                .astype(np.int32)
                .reshape(-1, 1, 2)
            ],
            True,
            (0, 255, 0),
            3
        )

        roi_preview, display_scale = self.resize_to_fit(
            preview,
            self.roi_preview_max_width,
            self.roi_preview_max_height
        )

        preview_h, _ = roi_preview.shape[:2]

        cv2.putText(
            roi_preview,
            "Drag INDICATOR box, ENTER/SPACE = save, ESC = cancel",
            (
                12,
                max(
                    25,
                    preview_h - 18
                )
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )

        window_name = "Enroll - {}".format(
            name
        )

        try:
            sx, sy, sw, sh = cv2.selectROI(
                window_name,
                roi_preview,
                showCrosshair=True,
                fromCenter=False
            )

            cv2.destroyWindow(
                window_name
            )

        except Exception as exc:
            try:
                cv2.destroyWindow(
                    window_name
                )
            except Exception:
                pass

            return EnrollInspectionResponse(
                False,
                "ROI window error: {}".format(exc),
                qr_value,
                "",
                pan,
                tilt
            )

        if sw <= 0 or sh <= 0:
            return EnrollInspectionResponse(
                False,
                "Enrollment cancelled.",
                qr_value,
                "",
                pan,
                tilt
            )

        inv_scale = 1.0 / display_scale

        x = int(round(sx * inv_scale))
        y = int(round(sy * inv_scale))
        w = int(round(sw * inv_scale))
        h = int(round(sh * inv_scale))

        x = max(0, min(full_w - 1, x))
        y = max(0, min(full_h - 1, y))
        w = max(1, min(full_w - x, w))
        h = max(1, min(full_h - y, h))

        indicator_points = self.bbox_corners(
            x,
            y,
            w,
            h
        )

        profile_dir = os.path.join(
            self.profiles_dir,
            self.safe_name(name)
        )

        os.makedirs(
            profile_dir,
            exist_ok=True
        )

        reference_path = os.path.join(
            profile_dir,
            "reference.jpg"
        )
        annotated_path = os.path.join(
            profile_dir,
            "reference_annotated.jpg"
        )
        profile_path = os.path.join(
            profile_dir,
            "profile.json"
        )

        # Original JPEG from the webcam is saved WITHOUT re-encoding.
        with open(
            reference_path,
            "wb"
        ) as file:
            file.write(
                jpeg_bytes
            )

        annotated = self.annotate(
            frame,
            name,
            qr_value,
            qr_points,
            indicator_points,
            aligned=True
        )

        cv2.imwrite(
            annotated_path,
            annotated,
            [cv2.IMWRITE_JPEG_QUALITY, 95]
        )

        profile = {
            "version": 2,
            "name": name,
            "created_at": datetime.now().isoformat(),
            "camera": {
                "width": int(full_w),
                "height": int(full_h),
            },
            "camera_reference": {
                "pan": float(pan),
                "tilt": float(tilt),
            },
            "qr": {
                "value": qr_value,
                "corners": qr_points.astype(float).tolist(),
            },
            "indicator": {
                "bbox": {
                    "x": int(x),
                    "y": int(y),
                    "w": int(w),
                    "h": int(h),
                },
                "corners": indicator_points.astype(float).tolist(),
            },
            "alignment": {
                "target": "indicator_center",
                "tolerance_px": float(self.tolerance_px),
                "margin_px": float(self.margin_px),
            },
            "files": {
                "reference": reference_path,
                "reference_annotated": annotated_path,
            }
        }

        with open(
            profile_path,
            "w"
        ) as file:
            json.dump(
                profile,
                file,
                indent=2
            )

        rospy.loginfo(
            "ENROLLED '%s' QR=%s pan=%.1f tilt=%.1f",
            name,
            qr_value,
            pan,
            tilt
        )

        return EnrollInspectionResponse(
            True,
            "Enrollment saved.",
            qr_value,
            profile_path,
            pan,
            tilt
        )

    # --------------------------------------------------------------
    # Tracking worker
    # --------------------------------------------------------------

    def tracking_worker(
        self,
        stop_event,
        profile,
        expected_qr,
        initial_qr,
        state,
        state_lock
    ):
        last_seq = -1
        previous_qr = np.asarray(
            initial_qr,
            dtype=np.float32
        )

        # Full-frame identity re-check no more than once per second
        # when local geometry tracking has failed.
        last_full_reacquire = 0.0

        rate = rospy.Rate(
            max(
                1.0,
                self.tracking_detection_hz
            )
        )

        while (
            not rospy.is_shutdown()
            and not stop_event.is_set()
        ):
            jpeg_bytes, seq, stamp = self.get_jpeg_packet()

            if (
                jpeg_bytes is None
                or seq == last_seq
            ):
                rate.sleep()
                continue

            last_seq = seq

            frame = self.decode_jpeg(
                jpeg_bytes,
                reduced=False
            )

            if frame is None:
                rate.sleep()
                continue

            qr_points = self.detect_qr_geometry(
                frame,
                previous_points=previous_qr,
                expected=expected_qr
            )

            # If local tracking fails, occasionally do a full pyzbar
            # decode to reacquire the EXPECTED QR.
            if qr_points is None:
                now = time.monotonic()

                if (
                    now - last_full_reacquire
                    >= 1.0
                ):
                    last_full_reacquire = now

                    _, qr_points = self.detect_qr_zbar(
                        frame,
                        expected_qr
                    )

            if qr_points is None:
                rate.sleep()
                continue

            previous_qr = qr_points

            try:
                indicator_points = self.project_indicator(
                    profile,
                    qr_points
                )
            except Exception:
                rate.sleep()
                continue

            packet = {
                "jpeg": jpeg_bytes,
                "seq": seq,
                "stamp": stamp,
                "frame": frame,
            }

            measurement = self.make_measurement(
                packet,
                qr_points,
                indicator_points
            )

            with state_lock:
                state["measurement"] = measurement
                state["last_seen_time"] = time.monotonic()

            rate.sleep()

    # --------------------------------------------------------------
    # INSPECT
    # --------------------------------------------------------------

    def run_callback(self, req):
        with self.arm_session_lock:
            try:
                return self._run_callback(req)
            finally:
                self._release_arm_session()

    def _run_callback(self, req):
        profile, profile_path = self.load_profile(
            req.name.strip()
        )

        if profile is None:
            return RunInspectionResponse(
                False,
                "Profile not found: {}".format(
                    profile_path
                ),
                "",
                "",
                "",
                "",
                "",
                0.0,
                0.0
            )

        name = profile["name"]
        expected_qr = profile["qr"]["value"]

        # Approximate starting pose only.
        self.move_to(
            profile["camera_reference"]["pan"],
            profile["camera_reference"]["tilt"],
            wait=True
        )

        rospy.loginfo(
            "INSPECT: quick QR check, then pan-only search if missing."
        )

        found = self.horizontal_qr_search(
            profile,
            expected_qr
        )

        if found is None:
            pan, tilt = self.get_pose()

            return RunInspectionResponse(
                False,
                "QR '{}' not found within {:.1f} seconds.".format(
                    expected_qr,
                    self.qr_search_timeout
                ),
                expected_qr,
                "",
                "",
                "",
                "",
                pan,
                tilt
            )

        try:
            initial_indicator = self.project_indicator(
                profile,
                found["qr"]
            )
        except Exception as exc:
            pan, tilt = self.get_pose()

            return RunInspectionResponse(
                False,
                "Indicator projection failed: {}".format(
                    exc
                ),
                expected_qr,
                "",
                "",
                "",
                "",
                pan,
                tilt
            )

        initial_measurement = self.make_measurement(
            found,
            found["qr"],
            initial_indicator
        )

        state = {
            "measurement": initial_measurement,
            "last_seen_time": time.monotonic(),
        }
        state_lock = threading.Lock()
        stop_event = threading.Event()

        detector_thread = threading.Thread(
            target=self.tracking_worker,
            args=(
                stop_event,
                profile,
                expected_qr,
                found["qr"],
                state,
                state_lock
            ),
            daemon=True
        )

        detector_thread.start()

        start_time = time.monotonic()
        last_controlled_seq = -1
        final_measurement = initial_measurement
        aligned = False
        reason = "Aligning"

        control_rate = rospy.Rate(
            max(
                1.0,
                self.tracking_control_hz
            )
        )

        try:
            while (
                not rospy.is_shutdown()
                and time.monotonic() - start_time
                <= self.alignment_timeout
            ):
                with state_lock:
                    measurement = state["measurement"]
                    last_seen_time = state["last_seen_time"]

                if (
                    time.monotonic()
                    - last_seen_time
                    > self.qr_search_timeout
                ):
                    reason = (
                        "QR lost for {:.1f} seconds."
                        .format(
                            self.qr_search_timeout
                        )
                    )
                    break

                if measurement is None:
                    control_rate.sleep()
                    continue

                final_measurement = measurement

                visibility = self.visibility_state(
                    measurement["frame"],
                    measurement["qr"],
                    measurement["indicator"]
                )

                self.publish_geometry(
                    measurement,
                    aligned=visibility["center_ok"]
                )

                # CONDITION 1:
                # normal success when the combined QR + indicator center
                # enters the configured center range.
                if visibility["center_ok"]:
                    aligned = True
                    reason = "Target inside center range"
                    break

                # Do not command twice from the same visual measurement.
                if (
                    measurement["seq"]
                    == last_controlled_seq
                ):
                    control_rate.sleep()
                    continue

                last_controlled_seq = measurement[
                    "seq"
                ]

                pan, tilt = self.get_pose()
                new_pan = pan
                new_tilt = tilt

                error_x = visibility["error_x"]
                error_y = visibility["error_y"]

                pan_step = self.tracking_step(error_x)
                tilt_step = self.tracking_step(error_y)

                # Same physical directions as the working WASD controls.
                if abs(error_x) > self.center_range_px:
                    if error_x > 0.0:
                        # RIGHT -> D -> pan +
                        new_pan = self.clamp(
                            pan + pan_step,
                            self.pan_min,
                            self.pan_max
                        )
                    else:
                        # LEFT -> A -> pan -
                        new_pan = self.clamp(
                            pan - pan_step,
                            self.pan_min,
                            self.pan_max
                        )

                if abs(error_y) > self.center_range_px:
                    if error_y < 0.0:
                        # TOP -> W -> tilt +
                        new_tilt = self.clamp(
                            tilt + tilt_step,
                            self.tilt_min,
                            self.tilt_max
                        )
                    else:
                        # BOTTOM -> S -> tilt -
                        new_tilt = self.clamp(
                            tilt - tilt_step,
                            self.tilt_min,
                            self.tilt_max
                        )

                if (
                    new_pan == pan
                    and new_tilt == tilt
                ):
                    # CONDITION 2:
                    # only when a servo cannot move farther, allow an
                    # off-center result if QR + indicator still fit inside
                    # the edge margin.
                    if visibility["edge_ok"]:
                        aligned = True
                        reason = (
                            "Servo limit reached, but QR + indicator "
                            "remain inside edge margin"
                        )
                    else:
                        reason = (
                            "Servo limit reached and QR/indicator "
                            "are too close to or outside image edge"
                        )
                    break

                if new_pan != pan:
                    self.send_servo(
                        self.pan_servo_id,
                        new_pan,
                        run_time=self.tracking_servo_run_time
                    )

                if new_tilt != tilt:
                    rospy.sleep(0.005)

                    self.send_servo(
                        self.tilt_servo_id,
                        new_tilt,
                        run_time=self.tracking_servo_run_time
                    )

                control_rate.sleep()

        finally:
            stop_event.set()
            detector_thread.join(
                timeout=1.0
            )

        if final_measurement is None:
            pan, tilt = self.get_pose()

            return RunInspectionResponse(
                False,
                reason,
                expected_qr,
                "",
                "",
                "",
                "",
                pan,
                tilt
            )

        pan, tilt = self.get_pose()

        # Keep the final box visible in the live UI briefly after tracking ends.
        self.publish_geometry(
            final_measurement,
            aligned=aligned
        )

        final_frame = final_measurement[
            "frame"
        ]
        final_indicator = final_measurement[
            "indicator"
        ]

        timestamp = datetime.now().strftime(
            "%Y%m%d_%H%M%S_%f"
        )

        run_dir = os.path.join(
            self.runs_dir,
            self.safe_name(name),
            timestamp
        )

        os.makedirs(
            run_dir,
            exist_ok=True
        )

        original_path = os.path.join(
            run_dir,
            "original.jpg"
        )
        annotated_path = os.path.join(
            run_dir,
            "annotated.jpg"
        )
        crop_path = os.path.join(
            run_dir,
            "indicator_crop.jpg"
        )
        result_path = os.path.join(
            run_dir,
            "result.json"
        )

        # Save webcam JPEG directly, preserving original compression.
        with open(
            original_path,
            "wb"
        ) as file:
            file.write(
                final_measurement["jpeg"]
            )

        annotated = self.annotate(
            final_frame,
            name,
            expected_qr,
            final_measurement["qr"],
            final_indicator,
            aligned
        )

        cv2.imwrite(
            annotated_path,
            annotated,
            [cv2.IMWRITE_JPEG_QUALITY, 95]
        )

        xs = final_indicator[:, 0]
        ys = final_indicator[:, 1]

        x1 = max(
            0,
            int(math.floor(xs.min()))
        )
        y1 = max(
            0,
            int(math.floor(ys.min()))
        )
        x2 = min(
            final_frame.shape[1],
            int(math.ceil(xs.max()))
        )
        y2 = min(
            final_frame.shape[0],
            int(math.ceil(ys.max()))
        )

        if x2 > x1 and y2 > y1:
            cv2.imwrite(
                crop_path,
                final_frame[
                    y1:y2,
                    x1:x2
                ],
                [cv2.IMWRITE_JPEG_QUALITY, 95]
            )
        else:
            crop_path = ""

        result = {
            "version": 3,
            "timestamp": datetime.now().isoformat(),
            "inspection_name": name,
            "qr_value": expected_qr,
            "aligned": bool(aligned),
            "message": reason,
            "camera_pose": {
                "pan": float(pan),
                "tilt": float(tilt),
            },
            "indicator_result": {
                "status": "NOT_ANALYZED"
            },
            "files": {
                "original": original_path,
                "annotated": annotated_path,
                "indicator_crop": crop_path,
            }
        }

        with open(
            result_path,
            "w"
        ) as file:
            json.dump(
                result,
                file,
                indent=2
            )

        return RunInspectionResponse(
            bool(aligned),
            reason,
            expected_qr,
            original_path,
            annotated_path,
            crop_path,
            result_path,
            pan,
            tilt
        )


if __name__ == "__main__":
    rospy.init_node(
        "stip_qr_inspection"
    )

    QRInspectionNode()
    rospy.spin()
