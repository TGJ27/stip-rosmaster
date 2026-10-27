#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64MultiArray, String


class FireViewer:
    """Smooth camera viewer with cached FIRE/SMOKE overlays."""

    def __init__(self):
        self.camera_topic = rospy.get_param(
            "~camera_topic",
            "/inspection_camera/image_raw/compressed"
        )
        self.detection_topic = rospy.get_param(
            "~detection_topic",
            "/fire_detection/detections"
        )
        self.status_topic = rospy.get_param(
            "~status_topic",
            "/fire_detection/status"
        )

        self.display_fps = max(
            1.0, float(rospy.get_param("~display_fps", 30.0))
        )
        self.preview_decode_scale = int(
            rospy.get_param("~preview_decode_scale", 2)
        )
        if self.preview_decode_scale not in (1, 2, 4, 8):
            self.preview_decode_scale = 2

        self.window_name = rospy.get_param(
            "~window_name",
            "STIP Fire Detection"
        )
        self.max_overlay_age = max(
            0.0,
            float(rospy.get_param("~max_overlay_age", 0.75))
        )

        cv2.setNumThreads(
            max(1, int(rospy.get_param("~opencv_threads", 1)))
        )

        self.jpeg_lock = threading.Lock()
        self.latest_jpeg = None
        self.camera_seq = 0
        self.first_callback_logged = False
        self.camera_subscriber = None

        self.data_lock = threading.Lock()
        self.latest_boxes = []
        self.source_width = 1920.0
        self.source_height = 1080.0
        self.detection_wall = 0.0
        self.status = "STARTING"

        # Detection/status subscriptions are independent of the camera viewer.
        rospy.Subscriber(
            self.detection_topic,
            Float64MultiArray,
            self.detection_callback,
            queue_size=1
        )
        rospy.Subscriber(
            self.status_topic,
            String,
            self.status_callback,
            queue_size=1
        )

        rospy.on_shutdown(self.shutdown)

    def camera_callback(self, msg):
        with self.jpeg_lock:
            self.latest_jpeg = bytes(msg.data)
            self.camera_seq += 1

        if not self.first_callback_logged:
            self.first_callback_logged = True
            rospy.loginfo(
                "Fire viewer receiving live JPEG frames (%d bytes).",
                len(msg.data)
            )

    def get_camera_packet(self):
        with self.jpeg_lock:
            return self.latest_jpeg, self.camera_seq

    def decode(self, jpeg_bytes):
        if not jpeg_bytes:
            return None

        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)

        flag = {
            1: cv2.IMREAD_COLOR,
            2: cv2.IMREAD_REDUCED_COLOR_2,
            4: cv2.IMREAD_REDUCED_COLOR_4,
            8: cv2.IMREAD_REDUCED_COLOR_8,
        }[self.preview_decode_scale]

        return cv2.imdecode(arr, flag)

    def detection_callback(self, msg):
        data = list(msg.data)
        if len(data) < 8:
            return

        src_w = float(data[2])
        src_h = float(data[3])
        count = max(0, int(round(data[7])))

        boxes = []
        for i in range(count):
            idx = 8 + i * 6
            if idx + 5 >= len(data):
                break

            boxes.append({
                "x1": float(data[idx]),
                "y1": float(data[idx + 1]),
                "x2": float(data[idx + 2]),
                "y2": float(data[idx + 3]),
                "conf": float(data[idx + 4]),
                "class_id": int(round(data[idx + 5])),
            })

        with self.data_lock:
            self.latest_boxes = boxes
            if src_w > 0:
                self.source_width = src_w
            if src_h > 0:
                self.source_height = src_h
            self.detection_wall = time.monotonic()

    def status_callback(self, msg):
        with self.data_lock:
            self.status = str(msg.data)

    @staticmethod
    def draw_boxes(frame, boxes, src_w, src_h):
        h, w = frame.shape[:2]
        sx = float(w) / max(1.0, src_w)
        sy = float(h) / max(1.0, src_h)

        for box in boxes:
            x1 = int(round(box["x1"] * sx))
            y1 = int(round(box["y1"] * sy))
            x2 = int(round(box["x2"] * sx))
            y2 = int(round(box["y2"] * sy))

            if box["class_id"] == 0:
                color = (0, 0, 255)
                label = "FIRE"
            else:
                color = (0, 220, 255)
                label = "SMOKE"

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                frame,
                "{} {:.0f}%".format(label, box["conf"] * 100.0),
                (max(0, x1), max(24, y1 - 7)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                color,
                2,
                cv2.LINE_AA
            )

    @staticmethod
    def draw_status(frame, status):
        if status == "FIRE":
            color = (0, 0, 255)
        elif status == "SMOKE":
            color = (0, 220, 255)
        elif status.startswith("POSSIBLE"):
            color = (0, 180, 255)
        elif status == "CLEAR":
            color = (0, 220, 0)
        else:
            color = (220, 220, 220)

        cv2.putText(
            frame,
            status,
            (18, 34),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            color,
            2,
            cv2.LINE_AA
        )

    def run(self):
        rospy.loginfo(
            "Waiting for first camera frame on %s ...",
            self.camera_topic
        )

        try:
            first = rospy.wait_for_message(
                self.camera_topic,
                CompressedImage,
                timeout=10.0
            )
        except rospy.ROSException:
            rospy.logerr(
                "No camera frame received within 10 seconds: %s",
                self.camera_topic
            )
            return

        with self.jpeg_lock:
            self.latest_jpeg = bytes(first.data)
            self.camera_seq = 1

        first_frame = self.decode(self.latest_jpeg)
        if first_frame is None:
            rospy.logerr("First fire-viewer JPEG could not be decoded.")
            return

        rospy.loginfo(
            "Fire viewer first frame decoded: %dx%d",
            first_frame.shape[1],
            first_frame.shape[0]
        )

        # Persistent subscription starts only after the camera is proven alive.
        self.camera_subscriber = rospy.Subscriber(
            self.camera_topic,
            CompressedImage,
            self.camera_callback,
            queue_size=1,
            buff_size=4 * 1024 * 1024,
            tcp_nodelay=True
        )

        cv2.namedWindow(
            self.window_name,
            cv2.WINDOW_NORMAL
        )

        current_frame = first_frame
        last_seq = -1
        rate = rospy.Rate(self.display_fps)

        while not rospy.is_shutdown():
            jpeg_bytes, seq = self.get_camera_packet()

            if jpeg_bytes is not None and seq != last_seq:
                decoded = self.decode(jpeg_bytes)
                if decoded is not None:
                    current_frame = decoded
                    last_seq = seq

            shown = current_frame.copy()

            now = time.monotonic()
            with self.data_lock:
                boxes = list(self.latest_boxes)
                src_w = float(self.source_width)
                src_h = float(self.source_height)
                status = self.status
                age = (
                    now - self.detection_wall
                    if self.detection_wall > 0.0
                    else 1e9
                )

            if age <= self.max_overlay_age:
                self.draw_boxes(shown, boxes, src_w, src_h)

            self.draw_status(shown, status)

            cv2.imshow(
                self.window_name,
                shown
            )

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                rospy.signal_shutdown("Fire viewer closed")
                break

            rate.sleep()

        cv2.destroyAllWindows()

    def shutdown(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == "__main__":
    rospy.init_node("stip_fire_viewer")
    FireViewer().run()
