#!/usr/bin/env python3
"""Read-only ROS status subscriber for the Tkinter launcher.

Emits JSON over stdout. Does not restart stages or own ROS hardware.
"""
import json
import sys
import time

import rospy
from std_msgs.msg import String


def on_stage(msg):
    try:
        data = json.loads(msg.data)
        print('@@STAGES@@' + json.dumps(data, separators=(',', ':')), flush=True)
    except (ValueError, TypeError) as exc:
        print('Invalid manager status: {}'.format(exc), file=sys.stderr, flush=True)


def main():
    rospy.init_node('stip_launcher_observer', anonymous=True, disable_signals=True)
    rospy.Subscriber('/stip/startup/modules', String, on_stage, queue_size=3)
    print('Waiting for manager stage state...', flush=True)
    while not rospy.is_shutdown():
        time.sleep(0.5)


if __name__ == '__main__':
    main()
