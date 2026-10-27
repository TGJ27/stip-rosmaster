#!/usr/bin/env python3
"""Non-destructive two-point in-scene correction for MLX90640 Celsius data.

Works on calibrated temperatures published by the sensor driver; never edits
sensor EEPROM or the original /thermal/image_raw ROS topic.
"""
import argparse
from datetime import datetime, timezone
import os
import sys
import time
import threading

import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def capture(args):
    rospy.init_node('thermal_temperature_reference_capture', anonymous=True)
    bridge = CvBridge()
    lock = threading.Lock()
    measurements = []
    last_info = None
    roi = max(1, min(15, args.roi))

    def receive(msg):
        nonlocal last_info
        try:
            frame = bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            if frame.shape != (24, 32):
                return
            if not np.isfinite(frame).all() or frame.min() < -40.0 or frame.max() > 300.0:
                return
            cy, cx = 12, 16
            half = roi // 2
            cropped = np.asarray(frame[max(0, cy-half):min(24, cy+half+1),
                                       max(0, cx-half):min(32, cx+half+1)], dtype=float)
            if cropped.size == 0:
                return
            measured = float(np.median(cropped))
            with lock:
                if len(measurements) >= args.frames:
                    return
                measurements.append(measured)
                last_info = float(msg.header.stamp.to_sec()) if msg.header.stamp else time.time()
        except Exception as exc:
            rospy.logwarn_throttle(5, 'Temperature capture error: %s', exc)

    sub = rospy.Subscriber(args.topic, Image, receive, queue_size=1, buff_size=1024*256)
    start = time.monotonic()
    try:
        while not rospy.is_shutdown():
            with lock:
                count = len(measurements)
            if count >= args.frames:
                break
            if time.monotonic() - start > args.timeout:
                raise RuntimeError('Only received {} of {} valid frames in {:.0f}s'.format(
                    count, args.frames, args.timeout))
            time.sleep(0.1)
    finally:
        sub.unregister()

    values = np.asarray(measurements, dtype=float)
    mean = float(np.mean(values))
    std = float(np.std(values))
    result = {
        'kind': 'mlx90640_temperature_reference_sample',
        'timestamp_utc': datetime.now(timezone.utc).isoformat(),
        'reference_c': float(args.reference),
        'sensor_median_mean_c': mean,
        'sensor_median_std_c': std,
        'frames': len(values),
        'roi_pixels': roi,
        'source_topic': args.topic,
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, 'w') as f:
        yaml.safe_dump(result, f, sort_keys=False)
    print('Reference {:.2f} C | sensor {:.3f} C | frame-to-frame SD {:.3f} C'.format(
        args.reference, mean, std))
    print('Saved sample:', args.output)
    if std > 0.8:
        print('WARNING: Large variation: hold the reference object stationary and repeat.')


def fit(args):
    samples = []
    for path in args.samples:
        with open(path) as f:
            value = yaml.safe_load(f)
        if not isinstance(value, dict) or value.get('kind') != 'mlx90640_temperature_reference_sample':
            raise ValueError('Not a temperature reference sample: {}'.format(path))
        samples.append((float(value['sensor_median_mean_c']), float(value['reference_c'])))
        if float(value['sensor_median_std_c']) > 0.8:
            print('WARNING: Unstable sample:', path)
    measured = np.array([s[0] for s in samples])
    references = np.array([s[1] for s in samples])
    if float(np.ptp(measured)) < 5 or float(np.ptp(references)) < 5:
        raise ValueError('Use two or more reference objects with at least 5 C separation.')
    gain, offset = [float(x) for x in np.polyfit(measured, references, 1)]
    if not 0.5 <= gain <= 1.5 or abs(offset) > 30:
        raise ValueError('Implausible correction gain/offset ({:.3f}, {:.3f}); inspect reference samples.'.format(gain, offset))
    rmse = float(np.sqrt(np.mean((measured * gain + offset - references)**2)))
    result = {
        'kind': 'mlx90640_display_temperature_correction',
        'gain': gain,
        'offset_c': offset,
        'fit_rmse_c': rmse,
        'source_samples': [os.path.abspath(p) for p in args.samples],
        'note': 'Global scene correction ONLY; factory EEPROM unmodified. Not safety certified.',
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, 'w') as f:
        yaml.safe_dump(result, f, sort_keys=False)
    print('Corrected Celsius = {:.6f} * raw Celsius + {:.6f}'.format(gain, offset))
    print('Reference fit RMSE: {:.3f} C'.format(rmse))
    print('Saved correction:', args.output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_subparsers(dest='action', required=True)
    cap = actions.add_parser('capture', help='Capture reference target at image center')
    cap.add_argument('--topic', default='/thermal/image_raw')
    cap.add_argument('--reference', type=float, required=True, help='Independent thermometer Celsius')
    cap.add_argument('--frames', type=int, default=30)
    cap.add_argument('--roi', type=int, default=5, help='Odd ROI size around center; default 5 pixels')
    cap.add_argument('--timeout', type=float, default=30)
    cap.add_argument('--output', required=True)
    fitp = actions.add_parser('fit', help='Fit global linear scene correction (no EEPROM edits)')
    fitp.add_argument('--samples', nargs='+', required=True)
    fitp.add_argument('--output', required=True)
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    if args.action == 'capture':
        if args.frames < 5 or args.roi < 1 or args.roi > 15 or args.timeout <= 0:
            parser.error('frames >= 5, 1 <= roi <= 15, timeout > 0 required')
        capture(args)
    else:
        if len(args.samples) < 2:
            parser.error('fit needs at least 2 different reference samples')
        fit(args)


if __name__ == '__main__':
    try:
        main()
    except (ValueError, RuntimeError, OSError) as exc:
        sys.exit('ERROR: {}'.format(exc))
