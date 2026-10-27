#!/usr/bin/env python3
"""Pure geometric and frame-analysis helpers for fire inspection.

H maps native thermal pixel center coordinates (x,y) -> RGB calibration pixels.
Never interpret colorized/upscaled preview as a temperature measurement.
"""
import math
import numpy as np
import cv2


def parse_detections(payload, max_age=0.75, min_confidence=0.25):
    """v16 fire detector layout: [camera_stamp,infer_stamp,w,h,active,conf,age,count,
    (x1,y1,x2,y2,score,class_id)*count]. Class 0 = fire, class 1 = smoke.
    """
    try:
        d = list(payload)
        if len(d) < 8 or not np.isfinite(d).all():
            return None
        camera_stamp, infer_stamp, width, height, _, _, age, count_float = d[:8]
        n = int(count_float)
        if n < 0 or n > 100 or count_float != n or len(d) != 8 + 6*n:
            return None
        if width <= 0 or height <= 0 or age < 0 or age > max_age or infer_stamp <= 0:
            return None
        boxes = []
        for i in range(n):
            x1, y1, x2, y2, conf, cls = d[8+i*6:14+i*6]
            if x2 <= x1 or y2 <= y1 or conf < min_confidence or cls not in (0.,1.):
                continue
            if x1 < 0 or y1 < 0 or x2 > width + 1 or y2 > height + 1:
                continue
            boxes.append({'box': (x1,y1,x2,y2), 'confidence': conf,
                          'class_id': int(cls), 'center': ((x1+x2)/2, (y1+y2)/2)})
        return {'stamp': infer_stamp, 'width': width, 'height': height, 'boxes': boxes}
    except (TypeError, ValueError, OverflowError):
        return None


def load_homography(config):
    h = np.asarray(config['H_thermal_to_rgb'], dtype=np.float64)
    width = int(config['rgb_width'])
    height = int(config['rgb_height'])
    if h.shape != (3,3) or not np.isfinite(h).all() or abs(np.linalg.det(h)) < 1e-10:
        raise ValueError('Invalid saved thermal RGB homography')
    if width <= 0 or height <= 0:
        raise ValueError('Invalid RGB calibration dimensions')
    if int(config.get('thermal_width',32)) != 32 or int(config.get('thermal_height',24)) != 24:
        raise ValueError('Expected native 32 x 24 thermal calibration')
    return h, width, height


def transform_for_rgb(h, ref_width, ref_height, width, height):
    return np.diag([width/float(ref_width), height/float(ref_height), 1.0]).dot(h)


def project(points, h):
    pts = np.asarray(points, dtype=np.float64).reshape(-1,2)
    hom = np.concatenate([pts,np.ones((len(pts),1))],axis=1)
    p = (h.dot(hom.T)).T
    if np.any(abs(p[:,2]) < 1e-9) or not np.isfinite(p).all():
        raise ValueError('Invalid projective coordinates')
    return p[:,:2]/p[:,2,None]


def thermal_center_rgb(h, rw, rh, width, height):
    return project([(15.5,11.5)], transform_for_rgb(h,rw,rh,width,height))[0]


def analyze_target(frame, bbox, h, rw, rh, width, height,
                   min_target_pixels=3, top_percentile=90.0, hot_pixel_threshold=65.0):
    """Warp RGB ROI into native 24x32 pixel coordinates, analyze real °C.

    Return None for insufficient overlap or invalid sensor data.
    """
    if frame.shape != (24,32) or not np.isfinite(frame).all():
        return None
    if frame.min() < -40 or frame.max() > 300:
        return None
    x1,y1,x2,y2 = bbox
    rgb_corners = [(x1,y1),(x2,y1),(x2,y2),(x1,y2)]
    try:
        inv = np.linalg.inv(transform_for_rgb(h,rw,rh,width,height))
        polygon = project(rgb_corners, inv)
        center = project([((x1+x2)/2, (y1+y2)/2)],inv)[0]
    except (np.linalg.LinAlgError,ValueError):
        return None
    if not (-0.5 <= center[0] <= 31.5 and -0.5 <= center[1] <= 23.5):
        return None
    if not np.isfinite(polygon).all() or np.max(np.abs(polygon)) > 10000:
        return None
    mask = np.zeros((24,32),dtype=np.uint8)
    cv2.fillPoly(mask, [np.rint(polygon).astype(np.int32)], 1)
    inside = mask.astype(bool)
    if inside.sum() < min_target_pixels:
        # small targets may cover subpixel area: insufficient resolution for confirmation
        return None
    vals = frame[inside]
    background = frame[~inside]
    if len(background) < 20:
        return None
    # A small percentile resists isolated dead/hot pixels; still report actual max.
    return {'max_c':float(vals.max()), 'hot_c':float(np.percentile(vals,top_percentile)),
            'background_c':float(np.median(background)), 'pixels':int(len(vals)),
            'center_native':tuple(float(v) for v in center),
            'hot_pixels':int(np.count_nonzero(vals>=hot_pixel_threshold))}


def passes_temperature(stats, threshold_c, delta_c, min_hot_pixels=2):
    return (stats is not None and stats['hot_c'] >= threshold_c and
            stats['hot_c'] - stats['background_c'] >= delta_c and
            stats['hot_pixels'] >= min_hot_pixels)


def control_step(error_px, image_dimension, gain_deg, max_step_deg, deadband_fraction, sign=1.0):
    half = 0.5 * image_dimension
    if half <= 0 or abs(error_px)/half <= deadband_fraction:
        return 0.0
    return sign * max(-max_step_deg, min(max_step_deg, gain_deg * error_px/half))
