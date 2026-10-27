#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import shutil
import subprocess

import rospy

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst

from sensor_msgs.msg import CompressedImage, CameraInfo


class InspectionCameraNode:
    """
    IMPORTANT:
    The Vision Mini TP already sends MJPEG.

    This node DOES NOT decode MJPEG to BGR during normal operation.
    It publishes the original JPEG frame bytes directly as CompressedImage.

    That removes:
      jpegdec
      BGR conversion
      NumPy frame copies
      cv_bridge conversion
      ~6.2 MB raw ROS image publication per 1080p frame
    """

    def __init__(self):
        Gst.init(None)

        self.device = rospy.get_param(
            "~device",
            "/dev/v4l/by-id/usb-SunplusIT_Inc_Vision_Mini_TP_Vision_Mini_TP-video-index0"
        )

        self.width = int(rospy.get_param("~width", 1920))
        self.height = int(rospy.get_param("~height", 1080))
        self.fps = int(rospy.get_param("~fps", 30))
        self.frame_id = rospy.get_param(
            "~frame_id",
            "inspection_camera_link"
        )

        # Camera image controls.
        self.apply_controls = bool(
            rospy.get_param("~apply_controls", True)
        )
        self.sharpness = int(
            rospy.get_param("~sharpness", 3)
        )
        self.exposure_auto_priority = int(
            rospy.get_param("~exposure_auto_priority", 0)
        )
        self.power_line_frequency = int(
            rospy.get_param("~power_line_frequency", 1)
        )
        self.focus_auto = bool(
            rospy.get_param("~focus_auto", True)
        )
        self.focus_absolute = int(
            rospy.get_param("~focus_absolute", 10)
        )

        # Compressed camera stream.
        self.image_pub = rospy.Publisher(
            "/inspection_camera/image_raw/compressed",
            CompressedImage,
            queue_size=1
        )

        # Calibration info is tiny. Latch it instead of publishing 30 times/s.
        self.info_pub = rospy.Publisher(
            "/inspection_camera/camera_info",
            CameraInfo,
            queue_size=1,
            latch=True
        )

        if self.apply_controls:
            self.apply_v4l2_controls()

        self.pipeline = None
        self.appsink = None
        self.start_pipeline()

        camera_info = CameraInfo()
        camera_info.width = self.width
        camera_info.height = self.height
        camera_info.header.frame_id = self.frame_id
        camera_info.distortion_model = "plumb_bob"

        # Calibration will populate K/D/R/P later.
        self.info_pub.publish(camera_info)

        rospy.loginfo(
            "Compressed inspection camera ready: %dx%d MJPEG @ %d FPS",
            self.width,
            self.height,
            self.fps
        )
        rospy.loginfo(
            "Publishing ORIGINAL webcam JPEG bytes on "
            "/inspection_camera/image_raw/compressed"
        )
        rospy.loginfo(
            "No JPEG decode / BGR conversion is performed by the camera node."
        )
        rospy.logwarn(
            "Camera calibration not loaded yet; camera_info intrinsics are zero."
        )

        rospy.on_shutdown(self.shutdown)

    def apply_v4l2_controls(self):
        exe = shutil.which("v4l2-ctl")

        if not exe:
            rospy.logwarn(
                "v4l2-ctl not found; skipping camera controls."
            )
            return

        controls = [
            "sharpness={}".format(self.sharpness),
            "exposure_auto_priority={}".format(
                self.exposure_auto_priority
            ),
            "power_line_frequency={}".format(
                self.power_line_frequency
            ),
            "focus_auto={}".format(
                1 if self.focus_auto else 0
            ),
        ]

        if not self.focus_auto:
            controls.append(
                "focus_absolute={}".format(
                    self.focus_absolute
                )
            )

        try:
            result = subprocess.run(
                [
                    exe,
                    "-d",
                    self.device,
                    "--set-ctrl=" + ",".join(controls)
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=3
            )

            if result.returncode == 0:
                rospy.loginfo(
                    "Applied camera controls: %s",
                    ",".join(controls)
                )
            else:
                rospy.logwarn(
                    "Some camera controls failed: %s",
                    result.stderr.strip()
                )

        except Exception as exc:
            rospy.logwarn(
                "Camera control setup failed: %s",
                exc
            )

    def pipeline_string(self):
        # No decode at all. Each V4L2 MJPEG frame arrives at appsink as JPEG.
        #
        # queue max-size-buffers=1 leaky=downstream:
        #   if Python/ROS is briefly slow, throw away stale frames.
        #
        # appsink max-buffers=1 drop=true sync=false:
        #   always expose the newest camera frame; never accumulate delay.
        return (
            "v4l2src device={device} do-timestamp=true ! "
            "image/jpeg,width={width},height={height},framerate={fps}/1 ! "
            "queue max-size-buffers=1 leaky=downstream ! "
            "appsink name=jpeg_sink "
            "emit-signals=false "
            "sync=false "
            "max-buffers=1 "
            "drop=true"
        ).format(
            device=self.device,
            width=self.width,
            height=self.height,
            fps=self.fps
        )

    def start_pipeline(self):
        pipeline_desc = self.pipeline_string()

        rospy.loginfo(
            "MJPEG passthrough pipeline: %s",
            pipeline_desc
        )

        self.pipeline = Gst.parse_launch(
            pipeline_desc
        )

        self.appsink = self.pipeline.get_by_name(
            "jpeg_sink"
        )

        if self.appsink is None:
            raise RuntimeError(
                "Could not create JPEG appsink"
            )

        result = self.pipeline.set_state(
            Gst.State.PLAYING
        )

        if result == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError(
                "Could not start MJPEG camera pipeline"
            )

        # Detect immediate startup errors.
        self.pipeline.get_state(
            2 * Gst.SECOND
        )

        bus = self.pipeline.get_bus()
        message = bus.timed_pop_filtered(
            100 * Gst.MSECOND,
            Gst.MessageType.ERROR
        )

        if message is not None:
            err, debug = message.parse_error()
            raise RuntimeError(
                "GStreamer camera error: {} | {}".format(
                    err,
                    debug
                )
            )

    def check_bus(self):
        bus = self.pipeline.get_bus()

        message = bus.pop_filtered(
            Gst.MessageType.ERROR
            | Gst.MessageType.EOS
        )

        if message is None:
            return None

        if message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()

            return "GStreamer ERROR: {} | {}".format(
                err,
                debug
            )

        return "GStreamer reached EOS"

    @staticmethod
    def jpeg_bytes_from_sample(sample):
        buffer = sample.get_buffer()

        ok, map_info = buffer.map(
            Gst.MapFlags.READ
        )

        if not ok:
            return None

        try:
            # JPEG is compressed, so this copy is far smaller than a
            # 1920x1080 BGR frame.
            return bytes(map_info.data)

        finally:
            buffer.unmap(map_info)

    def shutdown(self):
        if self.pipeline is not None:
            self.pipeline.set_state(
                Gst.State.NULL
            )

    def run(self):
        while not rospy.is_shutdown():
            error = self.check_bus()

            if error:
                rospy.logerr(error)
                break

            sample = self.appsink.emit(
                "try-pull-sample",
                500 * Gst.MSECOND
            )

            if sample is None:
                rospy.logwarn_throttle(
                    2.0,
                    "No MJPEG camera sample received"
                )
                continue

            jpeg_bytes = self.jpeg_bytes_from_sample(
                sample
            )

            if not jpeg_bytes:
                continue

            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id
            msg.format = "jpeg"
            msg.data = jpeg_bytes

            self.image_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node(
        "stip_inspection_camera"
    )

    InspectionCameraNode().run()
