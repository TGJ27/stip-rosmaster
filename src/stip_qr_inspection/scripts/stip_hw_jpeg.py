#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Persistent JetPack 5 MJPEG decoder; independent of the camera publisher.

Default hardware path was verified on this robot's 1920x1080 4:2:2 camera:
 appsrc ! jpegparse ! nvv4l2decoder mjpeg=1 ! nvvidconv
        ! video/x-raw,format=BGRx ! appsink

Only the ROS consumer uses this module. Hardware failures fall back to
OpenCV for the rest of that node's lifetime. No ROS topics are altered.
"""
import threading

import cv2
import numpy as np


class JpegDecoder:
    def __init__(self, backend='nvidia', reduced=False, label='jpeg'):
        backend = str(backend).strip().lower()
        if backend not in ('nvidia', 'opencv'):
            raise ValueError('decoder_backend must be nvidia or opencv')
        self.requested_backend = backend
        self.backend = backend
        self.reduced = bool(reduced)
        self.label = label
        self.lock = threading.Lock()
        self._gst = None
        self._pipeline = None
        self._src = None
        self._sink = None
        self._warned = False
        self._first_frame = True
        # Do not initialize the hardware while the QR worker is idle.

    def _open_hardware(self):
        import gi
        gi.require_version('Gst', '1.0')
        gi.require_version('GstVideo', '1.0')
        from gi.repository import Gst, GstVideo
        Gst.init(None)
        for name in ('jpegparse', 'nvv4l2decoder', 'nvvidconv'):
            if Gst.ElementFactory.find(name) is None:
                raise RuntimeError('GStreamer element not installed: ' + name)
        scale_caps = ',width=960,height=540' if self.reduced else ''
        description = (
            'appsrc name=jpeg_src is-live=true block=true format=time '
            'do-timestamp=true caps="image/jpeg,framerate=(fraction)30/1" '
            '! jpegparse ! nvv4l2decoder mjpeg=1 ! nvvidconv '
            '! video/x-raw,format=BGRx' + scale_caps + ' '
            '! appsink name=pixel_sink emit-signals=false sync=false '
            'max-buffers=1 drop=true enable-last-sample=false'
        )
        pipeline = Gst.parse_launch(description)
        src = pipeline.get_by_name('jpeg_src')
        sink = pipeline.get_by_name('pixel_sink')
        if not src or not sink:
            raise RuntimeError('GStreamer appsrc/appsink was not created')
        src.set_property('max-bytes', 2 * 1024 * 1024)
        result = pipeline.set_state(Gst.State.PLAYING)
        if result == Gst.StateChangeReturn.FAILURE:
            pipeline.set_state(Gst.State.NULL)
            raise RuntimeError('Failed to start NVIDIA JPEG decoding pipeline')
        self._gst = Gst
        self._gst_video = GstVideo
        self._pipeline = pipeline
        self._src = src
        self._sink = sink

    def _hardware_decode(self, jpeg):
        if self._pipeline is None:
            self._open_hardware()
        Gst = self._gst
        buffer = Gst.Buffer.new_allocate(None, len(jpeg), None)
        buffer.fill(0, jpeg)
        ret = self._src.emit('push-buffer', buffer)
        if ret != Gst.FlowReturn.OK:
            raise RuntimeError('GStreamer push-buffer: {}'.format(ret))
        timeout_ms = 1500 if self._first_frame else 350
        sample = self._sink.emit('try-pull-sample', timeout_ms * Gst.MSECOND)
        if sample is None:
            msg = self._pipeline.get_bus().pop_filtered(Gst.MessageType.ERROR)
            if msg:
                err, debug = msg.parse_error()
                raise RuntimeError('GStreamer decode error: {} [{}]'.format(err, debug))
            raise RuntimeError('Timed out waiting for NVIDIA decoded JPEG')
        caps = sample.get_caps()
        info = self._gst_video.VideoInfo()
        if not info.from_caps(caps):
            raise RuntimeError('Cannot read GStreamer BGRx layout')
        if info.finfo.name != 'BGRx':
            raise RuntimeError('Unexpected GStreamer format: {}'.format(info.finfo.name))
        width, height = int(info.width), int(info.height)
        row_stride = int(info.stride[0])
        buf = sample.get_buffer()
        ok, mapped = buf.map(Gst.MapFlags.READ)
        if not ok:
            raise RuntimeError('Cannot map GStreamer output to CPU')
        try:
            raw = bytes(mapped.data)
        finally:
            buf.unmap(mapped)
        # Rows may be padded: respect the stride from GstVideoInfo.
        minimum = (height - 1) * row_stride + width * 4
        if len(raw) < minimum:
            raise RuntimeError('Invalid output buffer: {} < {}'.format(len(raw), minimum))
        pixels = np.ndarray((height, width, 4), dtype=np.uint8,
                            buffer=raw, strides=(row_stride, 4, 1))
        # Existing inspection/optical-flow/QR code expects 3-channel BGR.
        frame = cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
        self._first_frame = False
        return frame

    def decode(self, jpeg):
        if not jpeg:
            return None
        with self.lock:
            if self.backend == 'nvidia':
                try:
                    return self._hardware_decode(bytes(jpeg))
                except Exception as exc:
                    if not self._warned:
                        self._warned = True
                        try:
                            import rospy
                            rospy.logwarn('%s NVIDIA JPEG decode failed (%s); '
                                          'switching to OpenCV fallback', self.label, exc)
                        except Exception:
                            print('{} NVIDIA JPEG decode failed: {}; OpenCV fallback'.format(
                                self.label, exc))
                    self.backend = 'opencv'
                    self.close()
            arr = np.frombuffer(jpeg, dtype=np.uint8)
            flag = cv2.IMREAD_REDUCED_COLOR_2 if self.reduced else cv2.IMREAD_COLOR
            return cv2.imdecode(arr, flag)

    def close(self):
        if self._pipeline is not None:
            self._pipeline.set_state(self._gst.State.NULL)
        self._pipeline = None
        self._src = None
        self._sink = None
