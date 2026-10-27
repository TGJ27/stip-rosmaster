"""Decode QR codes using the same ZBar-first approach as stip_qr_inspection.

Optional pyzbar/ZBar is preferred on the Jetson; OpenCV remains a fallback.
This module is ROS-free to allow offline regression tests.
"""
import cv2
import numpy as np

try:
    from pyzbar import pyzbar
except (ImportError, OSError):
    pyzbar = None


def _zbar_decode(frame):
    if pyzbar is None:
        return None
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
    try:
        decoded = pyzbar.decode(gray)
    except Exception:
        return None
    for barcode in decoded:
        if getattr(barcode, 'type', '') != 'QRCODE':
            continue
        data = barcode.data.decode('utf-8', errors='replace').strip()
        if not data:
            continue
        corners = getattr(barcode, 'polygon', None) or []
        if len(corners) == 4:
            points = np.asarray([[p.x, p.y] for p in corners], dtype=np.float32)
        else:
            box = barcode.rect
            x, y, w, h = box.left, box.top, box.width, box.height
            points = np.asarray([[x,y],[x+w,y],[x+w,y+h],[x,y+h]], dtype=np.float32)
        sums=points.sum(axis=1)
        diffs=np.diff(points,axis=1).reshape(-1)
        points=np.asarray([points[np.argmin(sums)],points[np.argmin(diffs)],
                           points[np.argmax(sums)],points[np.argmax(diffs)]],dtype=np.float32)
        return (data, points)
    return None


def decode_qr(frame):
    """Return (decoded value, 4+ QR corners) or (None, None)."""
    if frame is None or getattr(frame, 'size', 0) == 0:
        return None, None
    zbar = _zbar_decode(frame)
    if zbar is not None:
        return zbar
    detector = cv2.QRCodeDetector()
    for candidate in (frame, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame):
        try:
            value, corners, _ = detector.detectAndDecode(candidate)
            if value and corners is not None:
                return value, np.asarray(corners, dtype=np.float32).reshape(-1, 2)
        except cv2.error:
            continue
    return None, None
