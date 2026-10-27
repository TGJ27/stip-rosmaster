#!/usr/bin/env python3
"""Fire Inspection live viewer: AI boxes + aligned thermal + true target Celsius.

Display-only: reads existing image/detection/inspection topics; does not send
servo commands or participate in fire confirmation/alarm decisions.
"""
import os
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rospy
import rospkg
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float64MultiArray, String

# Import the unchanged pure geometry module from this ROS package.
sys.path.insert(0, os.path.join(rospkg.RosPack().get_path('stip_fire_inspection'), 'scripts'))
from fusion_core import analyze_target, load_homography, project, transform_for_rgb


class FireThermalViewer:
    def __init__(self):
        p = lambda name, default: rospy.get_param('~' + name, default)
        self.camera_topic = str(p('camera_topic', '/inspection_camera/image_raw/compressed'))
        self.detection_topic = str(p('detection_topic', '/fire_detection/detections'))
        self.thermal_topic = str(p('thermal_topic', '/thermal/image_raw'))
        self.fire_status_topic = str(p('fire_status_topic', '/fire_detection/status'))
        self.fusion_state_topic = str(p('fusion_state_topic', '/fire_inspection/state'))
        self.alpha = float(p('alpha', 0.50))
        self.display_fps = float(p('display_fps', 30))
        self.blur_sigma = float(p('blur_sigma', 2.0))
        self.auto_range = bool(p('auto_range', True))
        self.min_c = float(p('min_c', 15.0))
        self.max_c = float(p('max_c', 45.0))
        self.max_thermal_age = float(p('max_thermal_age_sec', 0.60))
        self.max_detection_age = float(p('max_detection_age_sec', 0.75))
        self.max_pair_age = float(p('max_pair_age_sec', 0.35))
        self.decode_scale = int(p('preview_decode_scale', 2))
        self.show_fov = bool(p('show_thermal_fov', True))
        self.window = str(p('window_name', 'STIP Fire + Thermal Inspection'))
        self.path = os.path.expanduser(str(p('alignment_file', '')))
        if not (0 <= self.alpha <= 1 and 1 <= self.display_fps <= 60 and self.blur_sigma >= 0
                and self.min_c < self.max_c and self.max_thermal_age > 0
                and self.max_detection_age > 0 and self.max_pair_age > 0
                and self.decode_scale in (1, 2, 4, 8)):
            raise ValueError('Invalid Fire+Thermal viewer settings')
        if not os.path.isfile(self.path):
            raise RuntimeError('Missing alignment calibration: {}. Use thermal_rgb_calibrate.launch'.format(self.path))
        with open(self.path) as f:
            self.H, self.ref_width, self.ref_height = load_homography(yaml.safe_load(f))
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.jpeg = None
        self.jpeg_seq = 0
        self.rgb_stamp = 0.0
        self.rgb_received = 0.0
        self.thermal = None
        self.thermal_seq = 0
        self.thermal_stamp = 0.0
        self.thermal_received = 0.0
        self.boxes = []
        self.box_stamp = 0.0
        self.box_recv = 0.0
        self.source_width = 1920.0
        self.source_height = 1080.0
        self.ai_status = 'STARTING'
        self.fusion_state = 'STARTING'
        self.current_range = None
        self.cached_heat = None
        self.cached_mask = None
        self.cached_seq = -1
        self.cached_wh = None
        self.overlay_pub = rospy.Publisher('/fire_inspection/image_overlay', Image, queue_size=1)
        cv2.setNumThreads(max(1, int(p('opencv_threads', 1))))
        rospy.Subscriber(self.camera_topic, CompressedImage, self.on_rgb, queue_size=1,
                         buff_size=4*1024*1024, tcp_nodelay=True)
        rospy.Subscriber(self.thermal_topic, Image, self.on_thermal, queue_size=1,
                         buff_size=256*1024, tcp_nodelay=True)
        rospy.Subscriber(self.detection_topic, Float64MultiArray, self.on_detection, queue_size=1)
        rospy.Subscriber(self.fire_status_topic, String, self.on_ai_status, queue_size=1)
        rospy.Subscriber(self.fusion_state_topic, String, self.on_fusion_state, queue_size=1)
        rospy.on_shutdown(self.shutdown)

    def on_rgb(self, msg):
        with self.lock:
            self.jpeg = bytes(msg.data)
            self.jpeg_seq += 1
            self.rgb_stamp = msg.header.stamp.to_sec()
            self.rgb_received = time.monotonic()

    def on_thermal(self, msg):
        try:
            frame = np.asarray(self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1'), dtype=np.float32)
            if frame.shape != (24, 32) or not np.isfinite(frame).all():
                return
            if frame.min() < -40 or frame.max() > 300:
                return
            with self.lock:
                self.thermal = frame.copy()
                self.thermal_seq += 1
                self.thermal_stamp = msg.header.stamp.to_sec()
                self.thermal_received = time.monotonic()
        except (ValueError, TypeError, RuntimeError) as exc:
            rospy.logwarn_throttle(3, 'Viewer thermal frame decode failed: %s', exc)

    def on_detection(self, msg):
        data = list(msg.data)
        if len(data) < 8:
            return
        try:
            if not np.isfinite(data).all():
                return
            srcw, srch = float(data[2]), float(data[3])
            n = int(data[7])
            if srcw <= 0 or srch <= 0 or n < 0 or n > 100 or 8+6*n != len(data):
                return
            boxes = []
            for i in range(n):
                k = 8 + 6*i
                x1,y1,x2,y2,confidence,class_id = data[k:k+6]
                if (x2 > x1 and y2 > y1 and 0 <= x1 < srcw and 0 <= y1 < srch and
                    x2 <= srcw+1 and y2 <= srch+1 and class_id in (0., 1.)):
                    boxes.append((float(x1),float(y1),float(x2),float(y2),float(confidence),int(class_id)))
            with self.lock:
                self.boxes = boxes
                self.source_width, self.source_height = srcw, srch
                self.box_stamp = float(data[1])
                self.box_recv = time.monotonic()
        except (TypeError, ValueError, OverflowError):
            return

    def on_ai_status(self, msg):
        with self.lock:
            self.ai_status = str(msg.data)

    def on_fusion_state(self, msg):
        with self.lock:
            self.fusion_state = str(msg.data)

    def render_thermal(self, frame, w, h):
        # The native temperature frame is never changed or republished here.
        if self.auto_range:
            lo, hi = np.percentile(frame, [5, 95])
            center = 0.5 * (lo + hi)
            span = max(float(hi-lo), 8.0)
            lo, hi = center-span/2, center+span/2
            if self.current_range is None:
                self.current_range = (lo, hi)
            else:
                old_lo, old_hi = self.current_range
                self.current_range = (old_lo*.85+lo*.15, old_hi*.85+hi*.15)
            lo, hi = self.current_range
        else:
            lo, hi = self.min_c, self.max_c
        H = transform_for_rgb(self.H, self.ref_width, self.ref_height, w, h)
        # Warping ORIGINAL 32x24 Celsius pixels directly preserves their geometry.
        data = cv2.warpPerspective(frame, H, (w,h), flags=cv2.INTER_LINEAR)
        cover = cv2.warpPerspective(np.ones((24,32), np.float32), H, (w,h),
                                    flags=cv2.INTER_LINEAR)
        if self.blur_sigma > 0:
            data = cv2.GaussianBlur(data, (0,0), self.blur_sigma)
            smooth_cover = cv2.GaussianBlur(cover, (0,0), self.blur_sigma)
        else:
            smooth_cover = cover
        corrected = data / np.maximum(smooth_cover, 1e-6)
        gray = np.uint8(np.clip((corrected-lo)*255.0/(hi-lo), 0, 255))
        heat = cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO)
        mask = cover > .5
        return heat, mask

    @staticmethod
    def put_line(frame, text, x, y, color, scale=0.52):
        x, y = int(max(2, x)), int(max(18, y))
        cv2.putText(frame, text, (x,y), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(frame, text, (x,y), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, color, 1, cv2.LINE_AA)

    def draw_boxes(self, shown, detections, src_w, src_h, tframe, tstamp, detect_stamp, valid_thermal):
        h,w=shown.shape[:2]
        sx,sy=w/max(1.,src_w),h/max(1.,src_h)
        matched = (valid_thermal and detect_stamp > 0 and tstamp > 0 and
                   abs(tstamp-detect_stamp) <= self.max_pair_age)
        for x1,y1,x2,y2,conf,cls in detections:
            ax1,ay1 = int(round(x1*sx)),int(round(y1*sy))
            ax2,ay2 = int(round(x2*sx)),int(round(y2*sy))
            rgb_box=(max(0,ax1),max(0,ay1),min(w-1,ax2),min(h-1,ay2))
            color=(0,0,255) if cls==0 else (0,220,255)
            cv2.rectangle(shown,(rgb_box[0],rgb_box[1]),(rgb_box[2],rgb_box[3]),color,2)
            self.put_line(shown, ('FIRE' if cls==0 else 'SMOKE')+' {:.0f}%'.format(conf*100),
                          rgb_box[0]+3, rgb_box[1]+20, color)
            if cls!=0:
                continue
            value='TEMP N/A'
            if valid_thermal and not matched:
                value='TEMP WAIT SYNC'
            if matched:
                stats=analyze_target(tframe,(x1,y1,x2,y2),self.H,self.ref_width,
                                     self.ref_height,src_w,src_h,min_target_pixels=3,
                                     top_percentile=90.0)
                if stats is not None:
                    # Maximum and robust P90 measured only inside native thermal ROI.
                    value='MAX {:.1f}C  P90 {:.1f}C'.format(stats['max_c'],stats['hot_c'])
                else:
                    value='TEMP OUT OF FOV / TOO SMALL'
            # Prefer inside the detected bounding box; small boxes use just below it.
            label_y=(rgb_box[1]+40 if rgb_box[3]-rgb_box[1]>=45
                     else min(h-8,rgb_box[3]+19))
            self.put_line(shown,value,rgb_box[0]+3,label_y,(255,255,255),0.46)

    def draw_footprint(self, shown):
        h,w=shown.shape[:2]
        try:
            H=transform_for_rgb(self.H,self.ref_width,self.ref_height,w,h)
            vertices=project([(-.5,-.5),(31.5,-.5),(31.5,23.5),(-.5,23.5)],H)
            if np.max(np.abs(vertices)) > 1e4:
                return
            pts=np.rint(vertices).astype(np.int32).reshape(-1,1,2)
            cv2.polylines(shown,[pts],True,(255,240,0),1,cv2.LINE_AA)
            center=project([(15.5,11.5)],H)[0]
            cx,cy=int(round(center[0])),int(round(center[1]))
            cv2.drawMarker(shown,(cx,cy),(255,255,255),cv2.MARKER_CROSS,10,1)
        except (ValueError,np.linalg.LinAlgError):
            pass

    def run(self):
        cv2.namedWindow(self.window,cv2.WINDOW_NORMAL)
        flags={1:cv2.IMREAD_COLOR,2:cv2.IMREAD_REDUCED_COLOR_2,
               4:cv2.IMREAD_REDUCED_COLOR_4,8:cv2.IMREAD_REDUCED_COLOR_8}
        rate=rospy.Rate(self.display_fps)
        cached_rgb=None
        last_jpeg_seq=-1
        rospy.loginfo('Fire+thermal viewer: RGB + raw MLX90640 + fire ROI temps at %.0f FPS',self.display_fps)
        while not rospy.is_shutdown():
            with self.lock:
                jpeg,seq=self.jpeg,self.jpeg_seq
                thermal,tseq,tstamp,trecv=self.thermal,self.thermal_seq,self.thermal_stamp,self.thermal_received
                boxes,srcw,srch,dstamp,drecv=list(self.boxes),self.source_width,self.source_height,self.box_stamp,self.box_recv
                ai,decision=self.ai_status,self.fusion_state
                camera_recent=self.rgb_received
            if jpeg is None:
                rate.sleep()
                continue
            if seq != last_jpeg_seq:
                candidate=cv2.imdecode(np.frombuffer(jpeg,np.uint8),flags[self.decode_scale])
                if candidate is not None:
                    cached_rgb=candidate
                    last_jpeg_seq=seq
            if cached_rgb is None:
                rate.sleep()
                continue
            shown=cached_rgb.copy()
            now=time.monotonic()
            rgb_live=now-camera_recent < 1.0
            thermal_live=thermal is not None and now-trecv <= self.max_thermal_age
            if thermal_live and rgb_live:
                wh=(shown.shape[1],shown.shape[0])
                if self.cached_seq!=tseq or self.cached_wh!=wh:
                    self.cached_heat,self.cached_mask=self.render_thermal(thermal,*wh)
                    self.cached_seq,self.cached_wh=tseq,wh
                if self.cached_mask is not None:
                    blended=cv2.addWeighted(shown,1-self.alpha,self.cached_heat,self.alpha,0)
                    shown[self.cached_mask]=blended[self.cached_mask]
            if self.show_fov:
                self.draw_footprint(shown)
            if now-drecv <= self.max_detection_age:
                self.draw_boxes(shown,boxes,srcw,srch,thermal,tstamp,dstamp,thermal_live)
            # RGB AI status is NOT fusion confirmation. Explicitly display both.
            cv2.rectangle(shown,(0,0),(min(shown.shape[1]-1,930),54),(0,0,0),-1)
            self.put_line(shown,'AI: {}   FUSION: {}'.format(ai,decision),9,21,(255,255,255),0.52)
            self.put_line(shown,'THERMAL {} | INFERNO AUTO={} | alpha={:.2f}'.format(
                'LIVE' if thermal_live else 'STALE / MISSING',self.auto_range,self.alpha),
                9,44,(255,255,255),0.46)
            cv2.imshow(self.window,shown)
            if self.overlay_pub.get_num_connections()>0:
                msg=self.bridge.cv2_to_imgmsg(shown,encoding='bgr8')
                msg.header.stamp=rospy.Time.now()
                msg.header.frame_id='inspection_camera_link'
                self.overlay_pub.publish(msg)
            if cv2.waitKey(1)&0xff in (ord('q'),ord('Q'),27):
                rospy.signal_shutdown('Fire viewer closed')
                break
            rate.sleep()
        cv2.destroyAllWindows()

    def shutdown(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__=='__main__':
    rospy.init_node('stip_fire_inspection_viewer')
    try:
        FireThermalViewer().run()
    except Exception as exc:
        rospy.logerr('Fire+thermal overlay viewer failed: %s',exc)
        raise SystemExit(1)
