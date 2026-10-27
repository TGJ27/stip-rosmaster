#!/usr/bin/env python3
"""Live 32x24 MLX90640-to-RGB registration with interactive numeric controls.

Thermal camera only. No gas imports, no sensor EEPROM writes, no alteration of Celsius.
WASD / mouse drag positions heatmap; side panel edits translation, scale,
rotation, opacity, and thermal horizontal flip (F). Press V to save H.
"""
import math
import os
import threading
import time
from datetime import datetime, timezone

import cv2
import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image


class ManualAlignment:
    FIELDS = [('dx', 'X offset', 'px', 1.0),
              ('dy', 'Y offset', 'px', 1.0),
              ('sx', 'Width scale', 'x', 0.02),
              ('sy', 'Height scale', 'x', 0.02),
              ('angle', 'Rotation', 'deg', 0.5),
              ('alpha', 'Opacity', '', 0.05)]

    def __init__(self):
        self.rgb_topic = rospy.get_param('~rgb_topic', '/inspection_camera/image_raw/compressed')
        self.thermal_topic = rospy.get_param('~thermal_topic', '/thermal/image_raw')
        self.output_topic = rospy.get_param('~output_topic', '/thermal/rgb_overlay')
        self.alignment_file = os.path.expanduser(rospy.get_param(
            '~alignment_file', os.path.expanduser('~/thermal_rgb_alignment.yaml')))
        self.fps = max(1.0, min(30.0, float(rospy.get_param('~display_fps', 20))))
        self.blur = max(0.0, float(rospy.get_param('~blur_sigma', 2)))
        self.min_c = float(rospy.get_param('~min_c', 15.0))
        self.max_c = float(rospy.get_param('~max_c', 45.0))
        self.auto_range = bool(rospy.get_param('~auto_range', True))
        self.min_span = max(2.0, float(rospy.get_param('~min_display_span_c', 8.0)))
        self.scale_low = None
        self.scale_high = None
        self.last_range_frame = None
        # A launch-level opacity of 0.50 wins over older saved alpha values;
        # saved homography, position, rotation, size, and flip are untouched.
        self.use_saved_alpha = bool(rospy.get_param('~use_saved_alpha', False))
        self.max_age = max(0.1, float(rospy.get_param('~max_thermal_age_sec', 0.6)))
        self.decode_scale = int(rospy.get_param('~decode_scale', 2))
        self.side_width = 350
        self.step_px = 5
        self.params = dict(dx=0.0, dy=0.0, sx=1.0, sy=1.0, angle=0.0,
                           alpha=float(rospy.get_param('~alpha', 0.4)))
        # Mirror the source thermal X-axis, never the RGB image or raw Celsius data.
        self.flip_lr = False
        self.temperature_file = os.path.expanduser(rospy.get_param('~temperature_file', ''))
        self.gain, self.offset = 1., 0.
        if self.temperature_file:
            with open(self.temperature_file) as f:
                cfg = yaml.safe_load(f)
            if cfg.get('kind') != 'mlx90640_display_temperature_correction':
                raise ValueError('Invalid temperature correction')
            self.gain, self.offset = float(cfg['gain']), float(cfg['offset_c'])
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.jpeg = None
        self.jpeg_stamp = rospy.Time(0)
        self.rgb_time = 0.0
        self.raw_temp = None
        self.thermal_time = 0.0
        self.rgb_bytes_last = None
        self.rgb_cache = None
        self.ref_width = 0
        self.ref_height = 0
        self.base_h = None
        self.loaded = False
        self.changed = True
        self.last_thermal_key = None
        self.warp_cache = None
        self.mask_cache = None
        self.warning = 'WASD move | F thermal flip L/R | V save'
        self.warning_until = time.monotonic() + 8.0
        self.editing = None
        self.edit_text = ''
        self.dragging = False
        self.drag_last = None
        self.window = 'STIP Thermal RGB Alignment | WASD / numeric sidebar'
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window, self.on_mouse)
        self.pub = rospy.Publisher(self.output_topic, Image, queue_size=1)
        rospy.Subscriber(self.rgb_topic, CompressedImage, self.on_rgb, queue_size=1,
                         buff_size=4*1024*1024)
        rospy.Subscriber(self.thermal_topic, Image, self.on_temp, queue_size=1,
                         buff_size=256*1024)
        rospy.loginfo('Manual fusion: WASD move, F flip thermal horizontally, V save')

    def notify(self, message, duration=5.0):
        self.warning = str(message)
        self.warning_until = time.monotonic() + duration
        rospy.loginfo('%s', message)

    def on_rgb(self, msg):
        with self.lock:
            self.jpeg = bytes(msg.data)
            self.jpeg_stamp = msg.header.stamp
            self.rgb_time = time.monotonic()

    def on_temp(self, msg):
        try:
            a = np.asarray(self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1'), dtype=np.float32)
            if a.shape != (24, 32) or not np.all(np.isfinite(a)) or a.min() < -40 or a.max() > 300:
                return
            with self.lock:
                self.raw_temp = a.copy()
                self.thermal_time = time.monotonic()
        except Exception as exc:
            rospy.logwarn_throttle(5, 'Fusion temperature frame rejected: %s', exc)

    def setup_for_size(self, w, h):
        if self.base_h is not None:
            return
        self.ref_width, self.ref_height = w, h
        # An initial guess only. The true optical FOV and mounting offset vary.
        target_w = 0.58*w
        target_h = min(0.75*h, target_w*24.0/32.0)
        dst = np.float32([[w/2-target_w/2, h/2-target_h/2],
                          [w/2+target_w/2, h/2-target_h/2],
                          [w/2+target_w/2, h/2+target_h/2],
                          [w/2-target_w/2, h/2+target_h/2]])
        src = np.float32([[-0.5, -0.5], [31.5, -0.5], [31.5, 23.5], [-0.5, 23.5]])
        self.base_h = cv2.getPerspectiveTransform(src, dst)
        self.load()
        self.changed = True

    def load(self):
        if not os.path.isfile(self.alignment_file):
            self.notify('No saved alignment yet - adjust then V to save')
            return
        try:
            with open(self.alignment_file) as f:
                cfg = yaml.safe_load(f)
            if int(cfg.get('thermal_width', 0)) != 32 or int(cfg.get('thermal_height', 0)) != 24:
                raise ValueError('Expected a 32x24 calibration')
            ref_w, ref_h = int(cfg['rgb_width']), int(cfg['rgb_height'])
            if ref_w <= 0 or ref_h <= 0:
                raise ValueError('Invalid reference dimensions')
            base = np.asarray(cfg.get('H_base_thermal_to_rgb', cfg['H_thermal_to_rgb']), dtype=float)
            if base.shape != (3, 3) or not np.isfinite(base).all() or abs(np.linalg.det(base)) < 1e-12:
                raise ValueError('Bad homography')
            self.base_h = base
            self.ref_width, self.ref_height = ref_w, ref_h
            defaults = dict(dx=0.,dy=0.,sx=1.,sy=1.,angle=0.,alpha=self.params['alpha'])
            defaults.update(cfg.get('manual_adjustments') or {})
            # Older calibration files without a flip flag remain unmirrored.
            self.flip_lr = bool(cfg.get('flip_horizontal', defaults.get('flip_lr', False)))
            for name, _, _, _ in self.FIELDS:
                if name == 'alpha' and not self.use_saved_alpha:
                    continue  # apply confirmed launch opacity by default
                self.params[name] = float(defaults[name])
                self.constrain(name)
            self.changed = True
            self.loaded = True
            self.notify('Loaded saved alignment: {}'.format(self.alignment_file))
        except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as exc:
            self.notify('Cannot load alignment: {}'.format(exc), 10.0)

    def effective_h(self, w, h):
        # Param values stored at reference RGB decode resolution; scale for display.
        src_center = np.float32([[[15.5, 11.5]]])
        center = cv2.perspectiveTransform(src_center, self.base_h)[0,0]
        cx, cy = float(center[0]), float(center[1])
        p = self.params
        theta = math.radians(p['angle'])
        cs, sn = math.cos(theta), math.sin(theta)
        adjust = np.array([[cs*p['sx'], -sn*p['sy'], 0],
                           [sn*p['sx'], cs*p['sy'], 0],
                           [0, 0, 1]], dtype=np.float64)
        translate = np.array([[1, 0, cx+p['dx']], [0, 1, cy+p['dy']], [0, 0, 1]], dtype=np.float64)
        uncenter = np.array([[1, 0, -cx], [0, 1, -cy], [0, 0, 1]], dtype=np.float64)
        scale_to_current = np.diag([float(w)/self.ref_width, float(h)/self.ref_height, 1.0])
        # Mirror about the native 32x24 thermal sensor center (x=15.5).
        # H saved below already includes this transform, so the production
        # overlay needs no separate flip or temperature-array modification.
        mirror = (np.array([[-1., 0., 31.], [0., 1., 0.], [0., 0., 1.]])
                  if self.flip_lr else np.eye(3))
        return scale_to_current.dot(translate).dot(adjust).dot(uncenter).dot(self.base_h).dot(mirror)

    def constrain(self, name):
        value = self.params[name]
        if not math.isfinite(value):
            raise ValueError('Finite number required')
        if name in ('sx', 'sy'):
            value = max(0.1, min(5.0, value))
        elif name == 'alpha':
            value = max(0., min(1., value))
        elif name == 'angle':
            value = max(-180., min(180., value))
        else:
            value = max(-5000., min(5000., value))
        self.params[name] = value
        self.changed = True

    def modify(self, name, value):
        self.params[name] = float(value)
        self.constrain(name)

    def save(self):
        if self.base_h is None:
            self.notify('No camera frames yet; cannot save')
            return
        h = self.effective_h(self.ref_width, self.ref_height)
        cfg = {'version': 2, 'kind': 'mlx90640_to_rgb_manual_registration',
               'created_utc': datetime.now(timezone.utc).isoformat(),
               'thermal_width': 32, 'thermal_height': 24,
               'rgb_width': self.ref_width, 'rgb_height': self.ref_height,
               'H_thermal_to_rgb': h.tolist(),
               'H_base_thermal_to_rgb': self.base_h.tolist(),
               'flip_horizontal': bool(self.flip_lr),
               'manual_adjustments': dict(
                   {k: float(v) for k, v in self.params.items()},
                   flip_lr=bool(self.flip_lr)),
               'note': 'Valid near the alignment target depth. Not a 3D parallax correction.'}
        try:
            parent = os.path.dirname(self.alignment_file)
            os.makedirs(parent, exist_ok=True)
            temp = self.alignment_file + '.tmp'
            with open(temp, 'w') as f:
                yaml.safe_dump(cfg, f, sort_keys=False)
            os.replace(temp, self.alignment_file)
            self.notify('Saved alignment: {}'.format(self.alignment_file), 7.0)
        except (OSError, yaml.YAMLError) as exc:
            self.notify('Cannot save calibration: {}'.format(exc), 10.0)

    def warp(self, temp, w, h):
        hmatrix = self.effective_h(w, h)
        corrected = np.asarray(temp*self.gain+self.offset, dtype=np.float32)
        covered = np.ones((24, 32), dtype=np.float32)
        mapped = cv2.warpPerspective(corrected, hmatrix, (w, h), flags=cv2.INTER_LINEAR)
        coverage = cv2.warpPerspective(covered, hmatrix, (w, h), flags=cv2.INTER_LINEAR)
        if self.blur > 0:
            mapped = cv2.GaussianBlur(mapped, (0, 0), self.blur)
            coverage_smooth = cv2.GaussianBlur(coverage, (0, 0), self.blur)
        else:
            coverage_smooth = coverage
        normalized = mapped / np.maximum(coverage_smooth, 1e-6)
        normalized = np.clip(normalized, float(corrected.min()), float(corrected.max()))
        # Auto contrast follows *native* 32x24 Celsius, not resampled pixels.
        # Update on a NEW sensor frame, not every WASD adjustment.
        if self.auto_range:
            if temp is not self.last_range_frame:
                lo, hi = np.percentile(corrected, [5, 95])
                middle = (float(lo) + float(hi)) / 2.0
                half = max((float(hi) - float(lo)) / 2.0, self.min_span / 2.0)
                next_low, next_high = middle - half, middle + half
                if self.scale_low is None:
                    self.scale_low, self.scale_high = next_low, next_high
                else:
                    self.scale_low = .85 * self.scale_low + .15 * next_low
                    self.scale_high = .85 * self.scale_high + .15 * next_high
                self.last_range_frame = temp
            low, high = self.scale_low, self.scale_high
        else:
            low, high = self.min_c, self.max_c
        gray = np.uint8(np.clip((normalized-low)*255/max(high-low, 0.01), 0, 255))
        rgb_heat = cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO)
        return rgb_heat, coverage > 0.55

    def draw_panel(self, img, w, h, fresh, thermal):
        bg = (28, 30, 34)
        cv2.rectangle(img, (w, 0), (w+self.side_width, h), bg, -1)
        cv2.putText(img, 'THERMAL / RGB ALIGN', (w+12, 31), cv2.FONT_HERSHEY_SIMPLEX,
                    .62, (243,243,243), 1, cv2.LINE_AA)
        cv2.putText(img, 'WASD = move | F = flip L/R', (w+12, 57), cv2.FONT_HERSHEY_SIMPLEX,
                    .46, (180,190,210), 1, cv2.LINE_AA)
        cv2.putText(img, 'V = save | Z/X = scale | 1/2/3 = step', (w+12, 77), cv2.FONT_HERSHEY_SIMPLEX,
                    .45, (180,190,210), 1, cv2.LINE_AA)
        cv2.putText(img, 'Click a value, type, ENTER', (w+12, 98), cv2.FONT_HERSHEY_SIMPLEX,
                    .45, (180,190,210), 1, cv2.LINE_AA)
        for i,(name,label,unit,step) in enumerate(self.FIELDS):
            y = 142 + i*55
            cv2.putText(img, label, (w+14,y-9), cv2.FONT_HERSHEY_SIMPLEX,
                        .51,(219,219,219),1,cv2.LINE_AA)
            cv2.rectangle(img, (w+12,y), (w+247,y+29),
                          (60,65,76) if self.editing != name else (90,95,106), -1)
            value = ('{:.2f}'.format(self.params[name]) if name not in ('dx','dy')
                     else '{:.1f}'.format(self.params[name]))
            if self.editing == name:
                value = self.edit_text + '_'
            cv2.putText(img, value+' '+unit, (w+22,y+20),cv2.FONT_HERSHEY_SIMPLEX,
                        .53,(250,250,250),1,cv2.LINE_AA)
            cv2.rectangle(img,(w+256,y),(w+288,y+29),(78,78,78),-1)
            cv2.rectangle(img,(w+297,y),(w+329,y+29),(78,78,78),-1)
            cv2.putText(img,'-',(w+266,y+21),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1)
            cv2.putText(img,'+',(w+305,y+21),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1)
        # Clickable horizontal mirror switch, also controlled by keyboard F.
        flip_y = 464
        flip_bg = (47, 103, 77) if self.flip_lr else (60, 65, 76)
        cv2.rectangle(img, (w+12, flip_y), (w+329, flip_y+40), flip_bg, -1)
        cv2.putText(img, '[X] FLIP LEFT/RIGHT (F)' if self.flip_lr
                    else '[ ] FLIP LEFT/RIGHT (F)',
                    (w+25, flip_y+26), cv2.FONT_HERSHEY_SIMPLEX,
                    .55, (255, 255, 255), 1, cv2.LINE_AA)
        help_lines = [
            'THERMAL CAMERA ONLY',
            'Width / height scale adjust camera field of view',
            'Rotation matches the physical mounting angle',
            'X/Y offset or WASD moves the overlay',
            'Place warm target at your working distance',
            'Thermal: {}'.format('OK' if fresh else 'WAITING/STALE'),
            'Original 32x24 Celsius measurements unchanged',
        ]
        for idx, line in enumerate(help_lines):
            yy = 533 + 22*idx
            cv2.putText(img, line, (w+12, yy), cv2.FONT_HERSHEY_SIMPLEX,
                        .37, (210,215,220), 1, cv2.LINE_AA)
        save_y = 703
        cv2.rectangle(img,(w+12,save_y),(w+self.side_width-12,save_y+39),(55,128,83),-1)
        cv2.putText(img,'SAVE ALIGNMENT (V)',(w+56,save_y+26),
                    cv2.FONT_HERSHEY_SIMPLEX,.54,(255,255,255),1,cv2.LINE_AA)

    def on_mouse(self,event,x,y,flags,userdata):
        w = self.rgb_cache.shape[1] if self.rgb_cache is not None else 0
        if not w:
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            if x < w:
                self.dragging = True
                self.drag_last = (x,y)
            else:
                relx = x-w
                for i,(name,_,_,step) in enumerate(self.FIELDS):
                    ry=142+i*55
                    if ry <= y <= ry+29:
                        if 12 <= relx <= 247:
                            self.editing=name
                            self.edit_text=''
                            return
                        if 256 <= relx <= 288:
                            self.modify(name, self.params[name]-step)
                            return
                        if 297 <= relx <= 329:
                            self.modify(name, self.params[name]+step)
                            return
                if 464 <= y <= 504 and 12 <= relx <= 329:
                    self.flip_lr = not self.flip_lr
                    self.changed = True
                    self.notify('Thermal mirror L/R: {}'.format('ON' if self.flip_lr else 'OFF'), 2)
                    return
                if 703 <= y <= 742 and 12 <= relx <= self.side_width-12:
                    self.save()
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            if self.drag_last is not None:
                dx=x-self.drag_last[0]
                dy=y-self.drag_last[1]
                self.modify('dx',self.params['dx']+dx*self.ref_width/w)
                self.modify('dy',self.params['dy']+dy*self.ref_height/self.rgb_cache.shape[0])
            self.drag_last=(x,y)
        elif event == cv2.EVENT_LBUTTONUP:
            self.dragging=False
            self.drag_last=None

    def handle_key(self, key):
        if key < 0:
            return True
        code = key & 0xFF
        if self.editing is not None:
            if code in (13,10):
                field=self.editing
                try:
                    self.modify(field,float(self.edit_text))
                except ValueError:
                    self.notify('Invalid numeric value',3)
                self.editing=None
            elif code==27:
                self.editing=None
            elif code in (8,127):
                self.edit_text=self.edit_text[:-1]
            elif chr(code) in '0123456789.-' and len(self.edit_text)<18:
                self.edit_text+=chr(code)
            return True
        c=chr(code).lower()
        if c=='q' or code==27:
            return False
        if c=='w': self.modify('dy',self.params['dy']-self.step_px)
        elif c=='s': self.modify('dy',self.params['dy']+self.step_px)
        elif c=='v': self.save()
        elif c=='f':
            self.flip_lr = not self.flip_lr
            self.changed = True
            self.notify('Thermal mirror L/R: {}'.format('ON' if self.flip_lr else 'OFF'), 2)
        elif c=='a': self.modify('dx',self.params['dx']-self.step_px)
        elif c=='d': self.modify('dx',self.params['dx']+self.step_px)
        elif c=='z': self.modify('sx',self.params['sx']-0.02); self.modify('sy',self.params['sy']-0.02)
        elif c=='x': self.modify('sx',self.params['sx']+0.02); self.modify('sy',self.params['sy']+0.02)
        elif c=='r': self.load()
        elif c=='0':
            self.params.update(dx=0.,dy=0.,sx=1.,sy=1.,angle=0.)
            self.flip_lr=False
            self.changed=True
        elif c=='1': self.step_px=1
        elif c=='2': self.step_px=5
        elif c=='3': self.step_px=20
        elif c in ('[',']'):
            self.modify('alpha',self.params['alpha']+(0.05 if c==']' else -0.05))
        elif c=='p': self.save()
        return True

    def run(self):
        rate=rospy.Rate(self.fps)
        while not rospy.is_shutdown():
            with self.lock:
                jpg=self.jpeg
                stamp=self.jpeg_stamp
                raw=self.raw_temp
                temp_time=self.thermal_time
                rgb_time=self.rgb_time
            if jpg is None:
                rate.sleep()
                continue
            if jpg is not self.rgb_bytes_last:
                flags = {1:cv2.IMREAD_COLOR,2:cv2.IMREAD_REDUCED_COLOR_2,
                         4:cv2.IMREAD_REDUCED_COLOR_4,8:cv2.IMREAD_REDUCED_COLOR_8}
                scale=self.decode_scale
                if scale not in flags:
                    raise ValueError('decode_scale must be one of 1,2,4,8')
                frame=cv2.imdecode(np.frombuffer(jpg,dtype=np.uint8),flags[scale])
                if frame is not None:
                    self.rgb_cache=frame
                    self.rgb_bytes_last=jpg
                    self.setup_for_size(frame.shape[1],frame.shape[0])
            if self.rgb_cache is None or self.base_h is None:
                rate.sleep()
                continue
            rgb=self.rgb_cache.copy()
            h,w=rgb.shape[:2]
            fresh=raw is not None and (time.monotonic()-temp_time)<self.max_age and \
                  (time.monotonic()-rgb_time)<1.0
            if fresh:
                key=(id(raw),w,h)
                if self.changed or key!=self.last_thermal_key:
                    self.warp_cache,self.mask_cache=self.warp(raw,w,h)
                    self.last_thermal_key=key
                    self.changed=False
                blended=cv2.addWeighted(rgb,1-self.params['alpha'],self.warp_cache,
                                        self.params['alpha'],0)
                rgb[self.mask_cache]=blended[self.mask_cache]
            msg=self.bridge.cv2_to_imgmsg(rgb,encoding='bgr8')
            msg.header.stamp=stamp if stamp != rospy.Time(0) else rospy.Time.now()
            msg.header.frame_id='inspection_camera_link'
            self.pub.publish(msg)
            if time.monotonic()<self.warning_until:
                cv2.rectangle(rgb,(0,0),(min(w,850),37),(0,0,0),-1)
                cv2.putText(rgb,self.warning[:90],(8,25),cv2.FONT_HERSHEY_SIMPLEX,.47,
                            (255,255,255),1,cv2.LINE_AA)
            canvas_h=max(h,770)
            display=np.zeros((canvas_h,w+self.side_width,3),np.uint8)
            display[:h,:w]=rgb
            self.draw_panel(display,w,canvas_h,fresh,raw)
            cv2.imshow(self.window,display)
            key=cv2.waitKeyEx(1)
            if not self.handle_key(key):
                break
            rate.sleep()
        cv2.destroyAllWindows()


if __name__=='__main__':
    rospy.init_node('stip_manual_thermal_rgb_alignment')
    try:
        ManualAlignment().run()
    except Exception as exc:
        rospy.logerr('Thermal-only alignment GUI failed: %s',exc)
        raise SystemExit(1)
