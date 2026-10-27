#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import threading
import time
from collections import deque

import cv2
import numpy as np
import rospy

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64MultiArray, Bool, String
from std_srvs.srv import SetBool, Trigger
from stip_rosmaster_msgs.msg import ArmJoint

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
    RunInspection,
)


class InspectionUI:
    def __init__(self):
        # Important: prevent OpenCV from consuming many cores just to display.
        self.opencv_threads = int(
            rospy.get_param(
                "~ui_opencv_threads",
                1
            )
        )
        cv2.setNumThreads(
            max(
                1,
                self.opencv_threads
            )
        )

        self.startup_guard_enabled = bool(rospy.get_param(
            "~startup_guard_enabled", False))
        self.bringup_ready = not self.startup_guard_enabled
        self.startup_last_heartbeat = 0.0
        self.startup_state = "Waiting for startup manager..."
        self.stage_states = {}
        self.retry_bringup_busy = False
        self.shutdown_busy = False
        self.shutdown_ack = False
        self.shutdown_error = ""
        if self.startup_guard_enabled:
            rospy.Subscriber("/stip/startup/bringup_ready", Bool,
                             self.bringup_ready_callback, queue_size=1)
            rospy.Subscriber("/stip/startup/status", String,
                             self.startup_status_callback, queue_size=1)
            rospy.Subscriber("/stip/startup/heartbeat", Bool,
                             self.startup_heartbeat_callback, queue_size=1)
            rospy.Subscriber("/stip/startup/modules", String,
                             self.startup_modules_callback, queue_size=1)
            self.retry_bringup_service = rospy.ServiceProxy(
                "/stip/startup/retry_bringup", Trigger)
            self.app_shutdown_service = rospy.ServiceProxy(
                "/stip/startup/shutdown", Trigger)

        self.camera_topic = rospy.get_param(
            "~camera_topic",
            "/inspection_camera/image_raw/compressed"
        )

        self.geometry_topic = rospy.get_param(
            "~geometry_topic",
            "/inspection/geometry"
        )

        # Decode/display rate for the local debug UI.
        # The future network dashboard receives the JPEG stream directly and
        # is not limited by this value.
        self.display_fps = float(
            rospy.get_param(
                "~display_fps",
                30.0
            )
        )

        # Do not paint old tracking boxes over a much newer live camera frame.
        # Backend geometry is an anchor, not the final displayed position.
        # We advance it to the current frame using optical flow.
        self.geometry_anchor_max_age = float(
            rospy.get_param(
                "~geometry_anchor_max_age",
                1.00
            )
        )
        self.tracked_box_max_age = float(
            rospy.get_param(
                "~tracked_box_max_age",
                0.50
            )
        )


        # Robust overlay tracker. Instead of trusting only the four QR
        # corners, track many high-contrast points around/inside the QR and
        # estimate motion with RANSAC. Bad flow is rejected instead of drawn.
        self.tracker_max_features = int(
            rospy.get_param("~tracker_max_features", 40)
        )
        self.tracker_min_features = int(
            rospy.get_param("~tracker_min_features", 8)
        )
        self.tracker_quality_level = float(
            rospy.get_param("~tracker_quality_level", 0.01)
        )
        self.tracker_min_distance = float(
            rospy.get_param("~tracker_min_distance", 5.0)
        )
        self.tracker_feature_padding_px = int(
            rospy.get_param("~tracker_feature_padding_px", 24)
        )
        self.tracker_ransac_threshold_px = float(
            rospy.get_param("~tracker_ransac_threshold_px", 3.0)
        )
        self.tracker_max_point_jump_px = float(
            rospy.get_param("~tracker_max_point_jump_px", 80.0)
        )
        self.tracker_max_corner_jump_px = float(
            rospy.get_param("~tracker_max_corner_jump_px", 95.0)
        )
        self.tracker_area_ratio_min = float(
            rospy.get_param("~tracker_area_ratio_min", 0.55)
        )
        self.tracker_area_ratio_max = float(
            rospy.get_param("~tracker_area_ratio_max", 1.80)
        )
        self.tracker_smoothing_alpha = float(
            rospy.get_param("~tracker_smoothing_alpha", 0.78)
        )
        self.tracker_hold_seconds = float(
            rospy.get_param("~tracker_hold_seconds", 0.22)
        )
        self.history_match_tolerance = float(
            rospy.get_param("~history_match_tolerance", 0.15)
        )

        # 2 = JPEG half-resolution decode (1920x1080 -> 960x540) for low CPU.
        # 1 = full 1920x1080 decode, higher CPU.
        self.preview_decode_scale = int(
            rospy.get_param(
                "~preview_decode_scale",
                2
            )
        )

        self.decoder_backend = rospy.get_param('~decoder_backend', 'nvidia')
        self.jpeg_decoder = JpegDecoder(
            self.decoder_backend,
            reduced=(self.preview_decode_scale == 2),
            label='inspection UI')
        rospy.loginfo('Inspection UI JPEG decoder requested: %s',
                      self.decoder_backend)

        # Arm values - WASD unchanged.
        self.pan_servo_id = int(
            rospy.get_param("~pan_servo_id", 1)
        )
        self.tilt_servo_id = int(
            rospy.get_param("~tilt_servo_id", 2)
        )

        self.pan = float(
            rospy.get_param("~pan_home", 90.0)
        )
        self.tilt = float(
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

        self.step = float(
            rospy.get_param("~step", 1.0)
        )
        self.run_time = int(
            rospy.get_param(
                "~servo_run_time",
                150
            )
        )

        self.jpeg_lock = threading.Lock()
        self.latest_camera_jpeg = None
        self.latest_camera_stamp = rospy.Time(0)
        self.camera_seq = 0

        # Small preview-frame history lets us find the frame on which the
        # backend actually detected the QR, then propagate its corners to the
        # CURRENT live frame using sparse optical flow.
        # ~1.5 seconds at 30 FPS. This lets a delayed backend geometry
        # anchor be matched to the actual preview frame it came from.
        self.frame_history = deque(maxlen=45)

        self.tracked_qr = None
        self.tracked_indicator = None
        self.tracked_prev_gray = None
        self.tracked_features = None
        self.tracked_stamp = 0.0
        self.last_valid_tracker_time = 0.0
        self.pending_geometry = None

        self.geometry_lock = threading.Lock()
        self.latest_geometry = None
        self.geometry_time = 0.0

        self.name = "indicator_1"
        self.name_buffer = ""
        self.editing_name = False

        self.status = (
            "READY - QR detection OFF until E or I"
        )

        self.busy = False
        self.operation = ""

        # C++ fire/smoke backend: actual inference is toggled via ROS SetBool.
        # We only draw fresh detector results; no box hold or smoothing.
        self.fire_lock = threading.Lock()
        self.fire_enabled = False
        self.fire_status = "DISABLED"
        self.fire_boxes = []
        self.fire_source_width = 1920.0
        self.fire_source_height = 1080.0
        self.fire_boxes_time = 0.0
        self.fire_toggle_busy = False
        self.fire_control_note = ""
        self.fire_overlay_max_age = max(0.01, float(
            rospy.get_param("~fire_overlay_max_age", 0.15)))
        self.fire_toggle_service = rospy.get_param(
            "~fire_toggle_service", "/fire_detection/set_enabled")
        self.fire_service = rospy.ServiceProxy(
            self.fire_toggle_service, SetBool)

        rospy.Subscriber(
            "/fire_detection/enabled", Bool,
            self.fire_enabled_callback, queue_size=1)
        rospy.Subscriber(
            "/fire_detection/status", String,
            self.fire_status_callback, queue_size=1)
        rospy.Subscriber(
            "/fire_detection/detections", Float64MultiArray,
            self.fire_detections_callback, queue_size=1)

        self.arm_pub = rospy.Publisher(
            "/TargetAngle",
            ArmJoint,
            queue_size=10
        )

        rospy.Subscriber(
            self.camera_topic,
            CompressedImage,
            self.camera_callback,
            queue_size=1,
            buff_size=4 * 1024 * 1024,
            tcp_nodelay=True
        )

        rospy.Subscriber(
            self.geometry_topic,
            Float64MultiArray,
            self.geometry_callback,
            queue_size=1,
            tcp_nodelay=True
        )

        rospy.Subscriber(
            "/ArmAngleUpdate",
            ArmJoint,
            self.arm_update_callback,
            queue_size=20,
            tcp_nodelay=True
        )

        # Never block UI startup on QR backend readiness. The QR worker is
        # independently supervised; service errors are handled when I/E runs.
        # A QR restart must not take down or prevent the camera/fire UI.

        self.enroll_srv = rospy.ServiceProxy(
            "/inspection/enroll",
            EnrollInspection
        )

        self.inspect_srv = rospy.ServiceProxy(
            "/inspection/run",
            RunInspection
        )

        self.ui_ready_pub = rospy.Publisher(
            "/stip/inspection_ui/ready", Bool, latch=True, queue_size=1)
        self.ui_frame_pub = rospy.Publisher(
            "/stip/inspection_ui/frame_ready", Bool, latch=True, queue_size=1)
        self.ui_frame_sent = False

        self.window = "STIP Inspection"

    def bringup_ready_callback(self, msg):
        self.bringup_ready = bool(msg.data)

    def startup_heartbeat_callback(self, msg):
        if msg.data:
            self.startup_last_heartbeat = time.monotonic()

    def system_ready(self):
        return (not self.startup_guard_enabled or
                (self.bringup_ready and
                 time.monotonic() - self.startup_last_heartbeat < 3.0))

    def startup_status_callback(self, msg):
        self.startup_state = str(msg.data)

    def startup_modules_callback(self, msg):
        try:
            document = json.loads(msg.data)
            self.stage_states = {key: value.get("state", "WAITING")
                                 for key, value in document.items()}
        except (ValueError, TypeError, AttributeError):
            pass

    def retry_bringup(self):
        if not self.startup_guard_enabled or self.retry_bringup_busy:
            return
        self.retry_bringup_busy = True
        self.status = "Manual bringup retry requested..."

        def do_retry():
            try:
                rospy.wait_for_service("/stip/startup/retry_bringup", timeout=2.0)
                result = self.retry_bringup_service()
                self.status = result.message
            except (rospy.ROSException, rospy.ServiceException) as exc:
                self.status = "Bringup retry unavailable: {}".format(exc)
            finally:
                self.retry_bringup_busy = False
        threading.Thread(target=do_retry, daemon=True).start()

    def request_app_shutdown(self):
        # Q is a supervisor shutdown command in staged mode. Closing the
        # window directly looks like a crash and triggers auto-restart.
        # Use a worker so the 30 FPS UI event loop stays responsive.
        if self.shutdown_busy or self.shutdown_ack:
            return
        self.shutdown_busy = True
        self.shutdown_error = ""
        self.status = "SHUTTING DOWN - waiting for supervisor acknowledgement"

        def do_shutdown():
            try:
                rospy.wait_for_service("/stip/startup/shutdown", timeout=1.0)
                response = self.app_shutdown_service()
                if response.success:
                    self.shutdown_ack = True
                else:
                    self.shutdown_error = response.message
            except (rospy.ROSException, rospy.ServiceException) as exc:
                self.shutdown_error = str(exc)
            finally:
                if not self.shutdown_ack:
                    self.status = ("SHUTDOWN UNAVAILABLE: " +
                                   self.shutdown_error[:90] +
                                   " - press Q to retry or Ctrl-C in terminal")
                self.shutdown_busy = False

        threading.Thread(target=do_shutdown, daemon=True).start()

    # --------------------------------------------------------------
    # Camera
    # --------------------------------------------------------------

    def camera_callback(self, msg):
        # NO JPEG decode in the ROS callback.
        with self.jpeg_lock:
            self.latest_camera_jpeg = msg.data
            self.latest_camera_stamp = msg.header.stamp
            self.camera_seq += 1

    def decode_jpeg(self, jpeg_bytes, reduced=False):
        # The decoder is persistent; reduced resolution is chosen at setup.
        return self.jpeg_decoder.decode(jpeg_bytes)

    def get_camera_packet(self):
        """Return the newest compressed camera frame and its sequence number."""
        with self.jpeg_lock:
            return (
                self.latest_camera_jpeg,
                int(self.camera_seq),
                self.latest_camera_stamp
            )

    def geometry_callback(self, msg):
        # Float64 layout:
        # timestamp, source_w, source_h, aligned,
        # 8 QR values, 8 indicator values.
        if len(msg.data) < 20:
            return

        data = list(msg.data)

        geometry = {
            "stamp": float(data[0]),
            "source_width": float(data[1]),
            "source_height": float(data[2]),
            "aligned": bool(data[3] > 0.5),
            "qr": np.asarray(
                data[4:12],
                dtype=np.float32
            ).reshape(4, 2),
            "indicator": np.asarray(
                data[12:20],
                dtype=np.float32
            ).reshape(4, 2),
        }

        with self.geometry_lock:
            self.pending_geometry = geometry
            self.latest_geometry = geometry
            self.geometry_time = time.monotonic()

    def get_pending_geometry(self):
        with self.geometry_lock:
            geometry = self.pending_geometry
            self.pending_geometry = None

        return geometry

    @staticmethod
    def gray_frame(frame):
        return cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

    def find_history_frame(self, stamp_sec):
        if not self.frame_history:
            return None

        item = min(
            self.frame_history,
            key=lambda entry: abs(
                entry["stamp"] - stamp_sec
            )
        )

        if abs(item["stamp"] - stamp_sec) > self.history_match_tolerance:
            return None

        return item

    @staticmethod
    def quad_area(points):
        points = np.asarray(
            points,
            dtype=np.float32
        ).reshape(-1, 2)

        if points.shape[0] != 4:
            return 0.0

        return abs(
            float(cv2.contourArea(points))
        )

    @staticmethod
    def finite_points(points):
        if points is None:
            return False

        return bool(
            np.all(
                np.isfinite(points)
            )
        )

    @staticmethod
    def transform_points(points, homography):
        if points is None or homography is None:
            return None

        try:
            transformed = cv2.perspectiveTransform(
                np.asarray(
                    points,
                    dtype=np.float32
                ).reshape(1, -1, 2),
                homography
            )[0]
        except Exception:
            return None

        if not InspectionUI.finite_points(transformed):
            return None

        return transformed

    def feature_mask_for_qr(self, gray, qr_points):
        h, w = gray.shape[:2]

        qr = np.asarray(
            qr_points,
            dtype=np.float32
        ).reshape(4, 2)

        x1 = max(
            0,
            int(np.floor(qr[:, 0].min()))
            - self.tracker_feature_padding_px
        )
        y1 = max(
            0,
            int(np.floor(qr[:, 1].min()))
            - self.tracker_feature_padding_px
        )
        x2 = min(
            w - 1,
            int(np.ceil(qr[:, 0].max()))
            + self.tracker_feature_padding_px
        )
        y2 = min(
            h - 1,
            int(np.ceil(qr[:, 1].max()))
            + self.tracker_feature_padding_px
        )

        if x2 <= x1 or y2 <= y1:
            return None

        mask = np.zeros_like(
            gray,
            dtype=np.uint8
        )

        cv2.rectangle(
            mask,
            (x1, y1),
            (x2, y2),
            255,
            -1
        )

        return mask

    def initialize_features(self, gray, qr_points):
        mask = self.feature_mask_for_qr(
            gray,
            qr_points
        )

        if mask is None:
            return None

        features = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=max(
                self.tracker_min_features,
                self.tracker_max_features
            ),
            qualityLevel=max(
                0.001,
                self.tracker_quality_level
            ),
            minDistance=max(
                2.0,
                self.tracker_min_distance
            ),
            mask=mask,
            blockSize=5,
            useHarrisDetector=False
        )

        if features is None:
            return None

        features = features.reshape(-1, 2)

        if len(features) < self.tracker_min_features:
            return None

        return features.astype(
            np.float32
        )

    def estimate_motion(
        self,
        old_gray,
        new_gray,
        old_features
    ):
        """
        Track many points, remove implausible jumps, then estimate a robust
        homography with RANSAC. Returns (H, new_inlier_features) or (None,None).
        """
        if (
            old_gray is None
            or new_gray is None
            or old_features is None
            or len(old_features) < self.tracker_min_features
        ):
            return None, None

        old_points = np.asarray(
            old_features,
            dtype=np.float32
        ).reshape(-1, 1, 2)

        new_points, status, error = cv2.calcOpticalFlowPyrLK(
            old_gray,
            new_gray,
            old_points,
            None,
            winSize=(31, 31),
            maxLevel=3,
            criteria=(
                cv2.TERM_CRITERIA_EPS
                | cv2.TERM_CRITERIA_COUNT,
                24,
                0.01
            )
        )

        if new_points is None or status is None:
            return None, None

        old_points = old_points.reshape(-1, 2)
        new_points = new_points.reshape(-1, 2)
        status = status.reshape(-1).astype(bool)

        if error is not None:
            error = error.reshape(-1)
            status &= np.isfinite(error)

        status &= np.all(
            np.isfinite(new_points),
            axis=1
        )

        old_valid = old_points[status]
        new_valid = new_points[status]

        if len(old_valid) < self.tracker_min_features:
            return None, None

        jumps = np.linalg.norm(
            new_valid - old_valid,
            axis=1
        )

        jump_ok = jumps <= self.tracker_max_point_jump_px
        old_valid = old_valid[jump_ok]
        new_valid = new_valid[jump_ok]

        if len(old_valid) < self.tracker_min_features:
            return None, None

        homography, inlier_mask = cv2.findHomography(
            old_valid,
            new_valid,
            method=cv2.RANSAC,
            ransacReprojThreshold=self.tracker_ransac_threshold_px,
            maxIters=500,
            confidence=0.995
        )

        if homography is None or inlier_mask is None:
            return None, None

        if not np.all(np.isfinite(homography)):
            return None, None

        inlier_mask = inlier_mask.reshape(-1).astype(bool)
        new_inliers = new_valid[inlier_mask]

        if len(new_inliers) < self.tracker_min_features:
            return None, None

        return homography, new_inliers.astype(np.float32)

    def validate_box_update(
        self,
        old_qr,
        new_qr,
        old_indicator,
        new_indicator,
        frame_shape
    ):
        """Reject exploding/inverted/implausible box transforms."""
        if not (
            self.finite_points(new_qr)
            and self.finite_points(new_indicator)
        ):
            return False

        old_qr_area = self.quad_area(old_qr)
        new_qr_area = self.quad_area(new_qr)
        old_ind_area = self.quad_area(old_indicator)
        new_ind_area = self.quad_area(new_indicator)

        if min(
            old_qr_area,
            new_qr_area,
            old_ind_area,
            new_ind_area
        ) < 4.0:
            return False

        qr_ratio = new_qr_area / old_qr_area
        indicator_ratio = new_ind_area / old_ind_area

        if not (
            self.tracker_area_ratio_min
            <= qr_ratio
            <= self.tracker_area_ratio_max
        ):
            return False

        if not (
            self.tracker_area_ratio_min
            <= indicator_ratio
            <= self.tracker_area_ratio_max
        ):
            return False

        corner_jump = np.linalg.norm(
            np.asarray(new_qr) - np.asarray(old_qr),
            axis=1
        )

        if float(corner_jump.max()) > self.tracker_max_corner_jump_px:
            return False

        # Reject non-convex QR shapes.
        qr_int = np.round(
            new_qr
        ).astype(np.int32)

        if not cv2.isContourConvex(qr_int):
            return False

        # Give some tolerance outside the preview while the servo is moving,
        # but reject truly wild coordinates.
        h, w = frame_shape[:2]
        pad_x = float(w) * 0.35
        pad_y = float(h) * 0.35

        all_points = np.vstack([
            new_qr,
            new_indicator
        ])

        if (
            all_points[:, 0].min() < -pad_x
            or all_points[:, 0].max() > float(w) + pad_x
            or all_points[:, 1].min() < -pad_y
            or all_points[:, 1].max() > float(h) + pad_y
        ):
            return False

        return True

    def smooth_points(self, old_points, new_points):
        alpha = min(
            1.0,
            max(
                0.0,
                self.tracker_smoothing_alpha
            )
        )

        return (
            alpha * np.asarray(new_points, dtype=np.float32)
            + (1.0 - alpha)
            * np.asarray(old_points, dtype=np.float32)
        )

    def set_tracker_state(
        self,
        gray,
        qr,
        indicator,
        stamp
    ):
        features = self.initialize_features(
            gray,
            qr
        )

        if features is None:
            return False

        self.tracked_prev_gray = gray
        self.tracked_qr = np.asarray(
            qr,
            dtype=np.float32
        )
        self.tracked_indicator = np.asarray(
            indicator,
            dtype=np.float32
        )
        self.tracked_features = features
        self.tracked_stamp = stamp
        self.last_valid_tracker_time = time.monotonic()

        return True

    def clear_tracker(self, keep_last_box=False):
        self.tracked_prev_gray = None
        self.tracked_features = None

        if not keep_last_box:
            self.tracked_qr = None
            self.tracked_indicator = None
            self.tracked_features = None
            self.tracked_prev_gray = None
            self.tracked_stamp = 0.0

    def seed_tracker_from_geometry(
        self,
        geometry,
        current_frame,
        current_gray,
        current_stamp
    ):
        if geometry is None:
            return

        age = current_stamp - geometry["stamp"]

        if age < -0.10 or age > self.geometry_anchor_max_age:
            return

        h, w = current_frame.shape[:2]
        source_w = max(
            1.0,
            geometry["source_width"]
        )
        source_h = max(
            1.0,
            geometry["source_height"]
        )

        scale_x = float(w) / source_w
        scale_y = float(h) / source_h

        anchor_qr = geometry["qr"].copy()
        anchor_indicator = geometry["indicator"].copy()

        anchor_qr[:, 0] *= scale_x
        anchor_qr[:, 1] *= scale_y
        anchor_indicator[:, 0] *= scale_x
        anchor_indicator[:, 1] *= scale_y

        history = self.find_history_frame(
            geometry["stamp"]
        )

        # Best case: geometry is tied to an old camera frame we still have.
        # Track MANY QR-region features from that frame directly to current.
        if history is not None:
            anchor_features = self.initialize_features(
                history["gray"],
                anchor_qr
            )

            if anchor_features is not None:
                homography, _ = self.estimate_motion(
                    history["gray"],
                    current_gray,
                    anchor_features
                )

                if homography is not None:
                    current_qr = self.transform_points(
                        anchor_qr,
                        homography
                    )
                    current_indicator = self.transform_points(
                        anchor_indicator,
                        homography
                    )

                    if (
                        current_qr is not None
                        and current_indicator is not None
                        and self.validate_box_update(
                            anchor_qr,
                            current_qr,
                            anchor_indicator,
                            current_indicator,
                            current_frame.shape
                        )
                    ):
                        self.set_tracker_state(
                            current_gray,
                            current_qr,
                            current_indicator,
                            current_stamp
                        )
                        return

        # If the anchor is very fresh, using it directly is safer than
        # inventing motion from a mismatched history frame.
        if abs(age) <= self.history_match_tolerance:
            self.set_tracker_state(
                current_gray,
                anchor_qr,
                anchor_indicator,
                current_stamp
            )

    def advance_tracker(
        self,
        current_frame,
        current_gray,
        current_stamp
    ):
        if (
            self.tracked_qr is None
            or self.tracked_indicator is None
            or self.tracked_prev_gray is None
            or self.tracked_features is None
        ):
            return

        homography, _ = self.estimate_motion(
            self.tracked_prev_gray,
            current_gray,
            self.tracked_features
        )

        if homography is None:
            # Keep the last valid box only for a very short time. Do not let
            # one bad flow estimate throw the overlay across the screen.
            self.clear_tracker(
                keep_last_box=True
            )
            return

        raw_qr = self.transform_points(
            self.tracked_qr,
            homography
        )
        raw_indicator = self.transform_points(
            self.tracked_indicator,
            homography
        )

        if (
            raw_qr is None
            or raw_indicator is None
            or not self.validate_box_update(
                self.tracked_qr,
                raw_qr,
                self.tracked_indicator,
                raw_indicator,
                current_frame.shape
            )
        ):
            self.clear_tracker(
                keep_last_box=True
            )
            return

        smooth_qr = self.smooth_points(
            self.tracked_qr,
            raw_qr
        )
        smooth_indicator = self.smooth_points(
            self.tracked_indicator,
            raw_indicator
        )

        # Re-seed features on the current QR region each frame. This prevents
        # long-lived bad points from accumulating drift.
        current_features = self.initialize_features(
            current_gray,
            smooth_qr
        )

        if current_features is None:
            self.clear_tracker(
                keep_last_box=True
            )
            return

        self.tracked_qr = smooth_qr
        self.tracked_indicator = smooth_indicator
        self.tracked_prev_gray = current_gray
        self.tracked_features = current_features
        self.tracked_stamp = current_stamp
        self.last_valid_tracker_time = time.monotonic()

    # --------------------------------------------------------------
    # Arm
    # --------------------------------------------------------------

    def arm_update_callback(self, msg):
        if msg.id == self.pan_servo_id:
            self.pan = float(msg.angle)
        elif msg.id == self.tilt_servo_id:
            self.tilt = float(msg.angle)

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))

    def send_servo(
        self,
        servo_id,
        angle
    ):
        msg = ArmJoint()
        msg.id = int(servo_id)
        msg.run_time = int(self.run_time)
        msg.angle = float(angle)
        msg.joints = []

        self.arm_pub.publish(msg)

        if servo_id == self.pan_servo_id:
            self.pan = float(angle)
        elif servo_id == self.tilt_servo_id:
            self.tilt = float(angle)

    def move_pan(self, delta):
        if self.busy:
            return

        self.pan = self.clamp(
            self.pan + delta,
            self.pan_min,
            self.pan_max
        )

        self.send_servo(
            self.pan_servo_id,
            self.pan
        )

    def move_tilt(self, delta):
        if self.busy:
            return

        self.tilt = self.clamp(
            self.tilt + delta,
            self.tilt_min,
            self.tilt_max
        )

        self.send_servo(
            self.tilt_servo_id,
            self.tilt
        )

    def home(self):
        if self.busy:
            return

        self.pan = float(
            rospy.get_param(
                "~pan_home",
                90.0
            )
        )
        self.tilt = float(
            rospy.get_param(
                "~tilt_home",
                0.0
            )
        )

        self.send_servo(
            self.pan_servo_id,
            self.pan
        )

        rospy.sleep(0.03)

        self.send_servo(
            self.tilt_servo_id,
            self.tilt
        )

        self.status = "HOME"

    # --------------------------------------------------------------
    # UI operations
    # --------------------------------------------------------------

    def do_enroll(self):
        if self.busy:
            return

        self.busy = True
        self.operation = "ENROLL"

        self.status = (
            "ENROLL: searching QR (max 5 sec)..."
        )

        try:
            response = self.enroll_srv(
                self.name
            )

            if response.success:
                self.status = (
                    "ENROLLED: {} / QR {}"
                    .format(
                        self.name,
                        response.qr_value
                    )
                )
            else:
                self.status = (
                    "ENROLL FAILED: "
                    + response.message
                )

        except Exception as exc:
            self.status = (
                "ENROLL ERROR: {}"
                .format(exc)
            )

        self.busy = False
        self.operation = ""

        cv2.namedWindow(
            self.window,
            cv2.WINDOW_NORMAL
        )

    def do_inspect(self):
        if self.busy:
            return

        self.busy = True
        self.operation = "INSPECT"

        self.status = (
            "INSPECT: searching QR (max 5 sec)..."
        )

        def worker():
            try:
                response = self.inspect_srv(
                    self.name
                )

                if response.success:
                    self.status = (
                        "PASS: target in range + captured"
                    )
                else:
                    self.status = (
                        "INSPECT FAILED: "
                        + response.message
                    )

            except Exception as exc:
                self.status = (
                    "INSPECT ERROR: {}"
                    .format(exc)
                )

            self.busy = False
            self.operation = ""
            self.tracked_qr = None
            self.tracked_indicator = None

        threading.Thread(
            target=worker,
            daemon=True
        ).start()

    # --------------------------------------------------------------
    # Name entry
    # --------------------------------------------------------------

    def handle_name_key(self, key):
        if key in (13, 10):
            value = self.name_buffer.strip()

            if value:
                self.name = value
                self.status = "NAME SET"

            self.name_buffer = ""
            self.editing_name = False
            return

        if key == 27:
            self.name_buffer = ""
            self.editing_name = False
            self.status = "NAME CANCELLED"
            return

        if key in (8, 127):
            self.name_buffer = (
                self.name_buffer[:-1]
            )
            return

        if (
            32 <= key <= 126
            and len(self.name_buffer) < 48
        ):
            self.name_buffer += chr(key)

    # --------------------------------------------------------------
    # Panel
    # --------------------------------------------------------------

    def draw_tracking_geometry(self, frame):
        if not (
            self.busy
            and self.operation == "INSPECT"
        ):
            return frame

        if (
            self.tracked_qr is None
            or self.tracked_indicator is None
        ):
            return frame

        # If optical flow was rejected, hold the last valid overlay only very
        # briefly, then hide it until the next backend geometry anchor.
        if self.tracked_prev_gray is None:
            if (
                time.monotonic()
                - self.last_valid_tracker_time
                > self.tracker_hold_seconds
            ):
                return frame
        elif (
            time.monotonic()
            - self.last_valid_tracker_time
            > self.tracked_box_max_age
        ):
            return frame

        qr = self.tracked_qr
        indicator = self.tracked_indicator

        cv2.polylines(
            frame,
            [
                np.round(qr)
                .astype(np.int32)
                .reshape(-1, 1, 2)
            ],
            True,
            (0, 255, 0),
            2
        )

        cv2.polylines(
            frame,
            [
                np.round(indicator)
                .astype(np.int32)
                .reshape(-1, 1, 2)
            ],
            True,
            (255, 255, 0),
            2
        )

        center = np.mean(
            indicator,
            axis=0
        )

        cv2.circle(
            frame,
            (
                int(round(center[0])),
                int(round(center[1]))
            ),
            5,
            (255, 255, 0),
            -1
        )

        label_point = tuple(
            np.round(
                indicator[0]
            ).astype(int)
        )

        cv2.putText(
            frame,
            "INDICATOR",
            (
                max(5, label_point[0]),
                max(22, label_point[1] - 6)
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 0),
            2,
            cv2.LINE_AA
        )

        h, w = frame.shape[:2]

        cv2.drawMarker(
            frame,
            (
                w // 2,
                h // 2
            ),
            (255, 255, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=24,
            thickness=2
        )

        return frame

    # --------------------------------------------------------------
    # Fire/smoke toggle and overlay (the C++ process owns inference)
    # --------------------------------------------------------------

    def fire_enabled_callback(self, msg):
        with self.fire_lock:
            was_enabled = self.fire_enabled
            self.fire_enabled = bool(msg.data)
            if not self.fire_enabled:
                self.fire_boxes = []
                self.fire_boxes_time = 0.0
                self.fire_status = "DISABLED"
            elif not was_enabled:
                self.fire_status = "CLEAR"

    def fire_status_callback(self, msg):
        with self.fire_lock:
            # Stale status messages must not visually re-enable a stopped node.
            if self.fire_enabled:
                self.fire_status = str(msg.data)

    def fire_detections_callback(self, msg):
        values = list(msg.data)
        if len(values) < 8:
            return
        count = max(0, int(round(values[7])))
        boxes = []
        for index in range(count):
            offset = 8 + 6 * index
            if offset + 5 >= len(values):
                break
            boxes.append((
                float(values[offset]),
                float(values[offset + 1]),
                float(values[offset + 2]),
                float(values[offset + 3]),
                float(values[offset + 4]),
                int(round(values[offset + 5]))
            ))
        with self.fire_lock:
            # Never resurrect boxes after F switched the detector OFF.
            if not self.fire_enabled:
                return
            self.fire_boxes = boxes
            self.fire_source_width = max(1.0, float(values[2]))
            self.fire_source_height = max(1.0, float(values[3]))
            self.fire_boxes_time = time.monotonic()

    def toggle_fire(self):
        with self.fire_lock:
            if self.fire_toggle_busy:
                return
            self.fire_toggle_busy = True
            target = not self.fire_enabled
            self.fire_control_note = "Switching {}...".format(
                "ON" if target else "OFF")

        def worker():
            try:
                rospy.wait_for_service(self.fire_toggle_service, timeout=2.0)
                response = self.fire_service(target)
                with self.fire_lock:
                    if response.success:
                        self.fire_enabled = target
                        self.fire_control_note = "FIRE {}".format(
                            "ON" if target else "OFF")
                        if not target:
                            self.fire_boxes = []
                            self.fire_boxes_time = 0.0
                            self.fire_status = "DISABLED"
                    else:
                        self.fire_control_note = "Fire toggle rejected: " + response.message
            except (rospy.ROSException, rospy.ServiceException) as exc:
                with self.fire_lock:
                    self.fire_control_note = "Fire control unavailable: {}".format(exc)
            finally:
                with self.fire_lock:
                    self.fire_toggle_busy = False

        threading.Thread(target=worker, daemon=True).start()

    def draw_fire_overlay(self, frame):
        now = time.monotonic()
        with self.fire_lock:
            if not self.fire_enabled:
                return frame
            if now - self.fire_boxes_time > self.fire_overlay_max_age:
                return frame
            boxes = list(self.fire_boxes)
            sw = self.fire_source_width
            sh = self.fire_source_height

        h, w = frame.shape[:2]
        sx = w / max(1.0, sw)
        sy = h / max(1.0, sh)
        for x1, y1, x2, y2, score, cls in boxes:
            if cls not in (0, 1):
                continue
            color = (0, 0, 255) if cls == 0 else (0, 215, 255)
            label = "FIRE" if cls == 0 else "SMOKE"
            p1 = (int(round(x1 * sx)), int(round(y1 * sy)))
            p2 = (int(round(x2 * sx)), int(round(y2 * sy)))
            cv2.rectangle(frame, p1, p2, color, 2)
            cv2.putText(
                frame, "{} {:.0f}%".format(label, score * 100),
                (max(0, p1[0]), max(24, p1[1] - 7)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
        return frame

    def draw_panel(self, frame):
        h, w = frame.shape[:2]
        panel_h = 144

        canvas = np.zeros(
            (
                h + panel_h,
                w,
                3
            ),
            dtype=np.uint8
        )

        canvas[:h] = frame

        display_name = (
            self.name_buffer + "_"
            if self.editing_name
            else self.name
        )

        line1 = (
            "Name: {}    Pan: {:.0f} deg    Tilt: {:.0f} deg"
            .format(
                display_name,
                self.pan,
                self.tilt
            )
        )

        line2 = (
            "W/S Tilt  A/D Pan  H Home  "
            "N Name  E Enroll  I Inspect  F Fire ON/OFF  Q Quit"
        )

        line3 = (
            "QR: {}    Status: {}"
            .format(
                "ACTIVE" if self.busy else "OFF",
                self.status
            )
        )

        with self.fire_lock:
            fire_enabled = self.fire_enabled
            fire_status = self.fire_status
            fire_note = self.fire_control_note
        line4 = "Fire: {}  |  {}  {}".format(
            "ON" if fire_enabled else "OFF",
            fire_status,
            fire_note)

        y = h + 24

        cv2.putText(
            canvas,
            line1,
            (12, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )

        cv2.putText(
            canvas,
            line2,
            (12, y + 34),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )

        cv2.putText(
            canvas,
            line3,
            (12, y + 68),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )

        cv2.putText(
            canvas,
            line4,
            (12, y + 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (0, 215, 255) if fire_enabled else (175, 175, 175),
            1,
            cv2.LINE_AA
        )

        return canvas

    # --------------------------------------------------------------
    # Main loop
    # --------------------------------------------------------------

    def run(self):
        cv2.namedWindow(
            self.window,
            cv2.WINDOW_NORMAL
        )
        # Window exists; this is stronger than mere ROS node registration.
        self.ui_ready_pub.publish(Bool(data=True))

        last_camera_seq = -1
        current_frame = None
        last_panel_signature = None

        rate = rospy.Rate(
            max(
                5.0,
                self.display_fps
            )
        )

        while not rospy.is_shutdown():
            if self.shutdown_ack:
                break
            camera_jpeg, camera_seq, camera_stamp = self.get_camera_packet()
            got_new_frame = False

            if (
                camera_jpeg is not None
                and camera_seq != last_camera_seq
            ):
                last_camera_seq = camera_seq

                # Direct half-resolution JPEG decode: no full-res decode + resize.
                decoded = self.decode_jpeg(
                    camera_jpeg,
                    reduced=(
                        self.preview_decode_scale == 2
                    )
                )

                if decoded is not None:
                    current_frame = decoded
                    got_new_frame = True
                    if not self.ui_frame_sent:
                        self.ui_frame_sent = True
                        self.ui_frame_pub.publish(Bool(data=True))

                    if self.busy and self.operation == "INSPECT":
                        # Only perform grayscale/optical-flow work while I runs.
                        current_stamp = camera_stamp.to_sec()
                        current_gray = self.gray_frame(current_frame)
                        self.frame_history.append({
                            "stamp": current_stamp,
                            "gray": current_gray,
                        })
                        pending = self.get_pending_geometry()
                        if pending is not None:
                            self.seed_tracker_from_geometry(
                                pending, current_frame,
                                current_gray, current_stamp)
                        else:
                            self.advance_tracker(
                                current_frame,
                                current_gray, current_stamp)
                    else:
                        self.tracked_qr = None
                        self.tracked_indicator = None
                        self.tracked_prev_gray = None
                        self.tracked_features = None
                        self.frame_history.clear()

            if current_frame is None:
                current_frame = np.zeros(
                    (540, 960, 3),
                    dtype=np.uint8
                )
                cv2.putText(
                    current_frame,
                    "Waiting for camera...",
                    (30, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA
                )

            # Redraw when a new camera frame arrives. During inspection,
            # geometry may change between display frames, so redraw at display rate.
            with self.fire_lock:
                fire_signature = (
                    self.fire_enabled, self.fire_status,
                    self.fire_control_note, self.fire_toggle_busy)
            panel_signature = (
                fire_signature,
                self.name,
                round(self.pan, 1),
                round(self.tilt, 1),
                self.status,
                self.busy,
                self.operation,
                self.editing_name,
                self.name_buffer,
                self.startup_state,
                self.system_ready(),
            )

            should_redraw = (
                got_new_frame
                or self.busy
                or panel_signature != last_panel_signature
            )

            if should_redraw:
                shown_frame = current_frame.copy()
                if self.startup_guard_enabled:
                    text = "STARTUP: " + self.startup_state[:96]
                    if not self.system_ready() and self.bringup_ready:
                        text = "STARTUP MANAGER OFFLINE - controls locked"
                    color = (50, 220, 50) if self.system_ready() else (0, 170, 255)
                    cv2.rectangle(shown_frame, (0, 0),
                                  (shown_frame.shape[1], 64), (0, 0, 0), -1)
                    cv2.putText(shown_frame, text[:100], (14, 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.46,
                                color, 1, cv2.LINE_AA)
                    cv2.putText(shown_frame, "R = manual bringup retry (robot stationary)",
                                (14, 49), cv2.FONT_HERSHEY_SIMPLEX,
                                0.44, (200, 200, 200), 1, cv2.LINE_AA)
                shown_frame = self.draw_tracking_geometry(
                    shown_frame
                )
                shown_frame = self.draw_fire_overlay(shown_frame)

                cv2.imshow(
                    self.window,
                    self.draw_panel(shown_frame)
                )

                last_panel_signature = panel_signature

            key = cv2.waitKey(1) & 0xFF

            if key != 255:
                if self.editing_name:
                    self.handle_name_key(key)

                elif key in (ord("q"), ord("Q")):
                    if self.startup_guard_enabled:
                        self.request_app_shutdown()
                    else:
                        break

                elif key in (ord("r"), ord("R")) and self.startup_guard_enabled:
                    self.retry_bringup()

                # Motor, arm, QR and fire controls are inhibited until bringup READY.
                elif (self.startup_guard_enabled and not self.system_ready() and
                      key in [ord(k) for k in "aAdDwWsShHeEiIfF"]):
                    self.status = "BRINGUP NOT READY - controls held; press R to retry"

                elif (self.startup_guard_enabled and
                      key in [ord(k) for k in "iIeE"] and
                      (self.stage_states.get("camera") != "READY" or
                       self.stage_states.get("qr") != "READY")):
                    self.status = "QR/CAMERA NOT READY - waiting for warmup"

                elif (self.startup_guard_enabled and
                      key in [ord(k) for k in "fF"] and
                      (self.stage_states.get("camera") != "READY" or
                       self.stage_states.get("fire") != "READY")):
                    self.status = "FIRE/CAMERA NOT READY - waiting for warmup"

                # WASD EXACTLY unchanged once bringup is healthy.
                elif key in (ord("a"), ord("A")):
                    self.move_pan(-self.step)

                elif key in (ord("d"), ord("D")):
                    self.move_pan(self.step)

                elif key in (ord("w"), ord("W")):
                    self.move_tilt(self.step)

                elif key in (ord("s"), ord("S")):
                    self.move_tilt(-self.step)

                elif key in (ord("h"), ord("H")):
                    self.home()

                elif (
                    key in (ord("n"), ord("N"))
                    and not self.busy
                ):
                    self.editing_name = True
                    self.name_buffer = ""
                    self.status = (
                        "TYPE NAME - ENTER save / ESC cancel"
                    )

                elif key in (ord("e"), ord("E")):
                    self.do_enroll()

                elif key in (ord("i"), ord("I")):
                    self.do_inspect()

                elif key in (ord("f"), ord("F")):
                    self.toggle_fire()

            rate.sleep()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node(
        "stip_inspection_ui"
    )

    InspectionUI().run()
