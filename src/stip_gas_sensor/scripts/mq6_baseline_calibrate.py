#!/usr/bin/env python3
"""Request existing gas_bridge /gas_sensor/calibrate Trigger and monitor status.

No direct POP32i UART access; no EEPROM alteration. Produces a clean-air ADC
baseline for relative comparisons only, NOT a validated ppm or %LEL reading.
"""
import argparse
import sys
import threading
import time
import rospy
from std_msgs.msg import String, Int32
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=90.0)
    parser.add_argument('--yes', action='store_true', help='Confirm clean air and stabilized sensor')
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    if args.timeout <= 0:
        parser.error('timeout must be positive')
    if not args.yes:
        print('MQ-6 must be warmed up/stabilized in known clean air, away from gas sources.')
        print('Baseline is relative raw ADC only. NOT a ppm, LEL, or safety calibration.')
        if input('Start clean-air baseline? [y/N] ').lower().strip() != 'y':
            print('Cancelled')
            return
    rospy.init_node('stip_mq6_baseline_cli', anonymous=True)
    lock=threading.Lock()
    state={'status':None, 'baseline':None}
    def status_cb(msg):
        with lock:
            state['status']=msg.data
    def baseline_cb(msg):
        with lock:
            state['baseline']=msg.data
    rospy.Subscriber('/gas_sensor/calibration_status', String, status_cb, queue_size=1)
    rospy.Subscriber('/gas_sensor/baseline', Int32, baseline_cb, queue_size=1)
    rospy.wait_for_service('/gas_sensor/calibrate', timeout=10)
    response = rospy.ServiceProxy('/gas_sensor/calibrate', Trigger)()
    print(response.message)
    if not response.success:
        raise RuntimeError(response.message)
    started=time.monotonic()
    while not rospy.is_shutdown() and time.monotonic()-started<args.timeout:
        with lock:
            status=state['status']
            baseline=state['baseline']
        if status == 'CALIBRATED_RELATIVE_ONLY':
            print('SUCCESS: saved MQ-6 raw ADC baseline =', baseline)
            print('Persistent file: ~/.ros/stip_mq6_baseline.json')
            return
        if status and status.startswith('FAILED'):
            raise RuntimeError('MQ-6 baseline failed: '+status)
        time.sleep(.2)
    raise RuntimeError('Timed out waiting for baseline. Inspect /gas_sensor/calibration_status.')


if __name__=='__main__':
    try:
        main()
    except (RuntimeError, rospy.ROSException, rospy.ServiceException) as exc:
        sys.exit('ERROR: '+str(exc))
