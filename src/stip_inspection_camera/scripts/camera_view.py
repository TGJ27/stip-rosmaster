#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage


class CameraView:
    """Live viewer for the native compressed inspection camera stream."""

    def __init__(self):
        self.topic = rospy.get_param(
            "~camera_topic",
            "/inspection_camera/image_raw/compressed"
        )
        self.window_name = rospy.get_param(
            "~window_name",
            "STIP Inspection Camera"
        )
        self.display_fps = max(
            1.0, float(rospy.get_param("~display_fps", 30.0))
        )
        self.preview_decode_scale = int(
            rospy.get_param("~preview_decode_scale", 2)
        )
        if self.preview_decode_scale not in (1, 2, 4, 8):
            self.preview_decode_scale = 2

        cv2.setNumThreads(
            max(1, int(rospy.get_param("~opencv_threads", 1)))
        )

        self.lock = threading.Lock()
        self.latest_jpeg = None
        self.seq = 0
        self.first_callback_logged = False
        self.subscriber = None

        rospy.on_shutdown(self.shutdown)

    def camera_callback(self, msg):
        with self.lock:
            self.latest_jpeg = bytes(msg.data)
            self.seq += 1

        if not self.first_callback_logged:
            self.first_callback_logged = True
            rospy.loginfo(
                "Camera viewer receiving live JPEG frames (%d bytes).",
                len(msg.data)
            )

    def get_latest(self):
        with self.lock:
            return self.latest_jpeg, self.seq

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

    def run(self):
        # The working QR inspection UI only begins its display loop after the
        # camera/backend are ready. Do the same here. This also uses the exact
        # wait_for_message path already verified manually on the Jetson.
        rospy.loginfo(
            "Waiting for first camera frame on %s ...",
            self.topic
        )

        try:
            first = rospy.wait_for_message(
                self.topic,
                CompressedImage,
                timeout=10.0
            )
        except rospy.ROSException:
            rospy.logerr(
                "No camera frame received within 10 seconds: %s",
                self.topic
            )
            return

        with self.lock:
            self.latest_jpeg = bytes(first.data)
            self.seq = 1

        first_frame = self.decode(self.latest_jpeg)
        if first_frame is None:
            rospy.logerr("First camera JPEG could not be decoded.")
            return

        rospy.loginfo(
            "First camera frame decoded: %dx%d",
            first_frame.shape[1],
            first_frame.shape[0]
        )

        # Create the persistent subscriber only after the publisher is proven
        # alive. This avoids the startup race seen in the simple viewer.
        self.subscriber = rospy.Subscriber(
            self.topic,
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
            jpeg_bytes, seq = self.get_latest()

            if jpeg_bytes is not None and seq != last_seq:
                decoded = self.decode(jpeg_bytes)
                if decoded is not None:
                    current_frame = decoded
                    last_seq = seq

            cv2.imshow(
                self.window_name,
                current_frame
            )

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                rospy.signal_shutdown("Camera viewer closed")
                break

            rate.sleep()

        cv2.destroyAllWindows()

    def shutdown(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == "__main__":
    rospy.init_node("stip_inspection_camera_view")
    CameraView().run()
