#!/usr/bin/env python3

import sys
import termios
import tty

import rospy
from stip_rosmaster_msgs.msg import ArmJoint


class CameraArmTeleop:
    def __init__(self):
        # Servo assignment
        self.pan_id = rospy.get_param("~pan_servo_id", 1)
        self.tilt_id = rospy.get_param("~tilt_servo_id", 2)

        # Temporary generic limits.
        # Replace these with the real mechanical limits after testing.
        self.pan_min = rospy.get_param("~pan_min", 0.0)
        self.pan_max = rospy.get_param("~pan_max", 180.0)
        self.tilt_min = rospy.get_param("~tilt_min", 0.0)
        self.tilt_max = rospy.get_param("~tilt_max", 180.0)

        # Current/start position
        self.pan = rospy.get_param("~pan_home", 90.0)
        self.tilt = rospy.get_param("~tilt_home", 0.0)

        # Movement tuning
        self.step = rospy.get_param("~step", 5.0)
        self.run_time = rospy.get_param("~run_time", 200)

        self.pub = rospy.Publisher("/TargetAngle", ArmJoint, queue_size=10)
        rospy.sleep(0.5)

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))

    def send_servo(self, servo_id, angle):
        msg = ArmJoint()
        msg.id = int(servo_id)
        msg.run_time = int(self.run_time)
        msg.angle = float(angle)
        msg.joints = []
        self.pub.publish(msg)

    def send_pan(self):
        self.send_servo(self.pan_id, self.pan)

    def send_tilt(self):
        self.send_servo(self.tilt_id, self.tilt)

    def home(self):
        self.pan = rospy.get_param("~pan_home", 90.0)
        self.tilt = rospy.get_param("~tilt_home", 0.0)
        self.send_pan()
        rospy.sleep(0.05)
        self.send_tilt()

    def print_status(self):
        sys.stdout.write(
            "\rPAN(ID %d): %6.1f deg   TILT(ID %d): %6.1f deg   "
            "W/S=up/down A/D=left/right H=home Q=quit      "
            % (self.pan_id, self.pan, self.tilt_id, self.tilt)
        )
        sys.stdout.flush()

    def run(self):
        print("")
        print("STIP Camera Arm Teleop")
        print("----------------------")
        print("A = servo 1 left")
        print("D = servo 1 right")
        print("W = servo 2 up")
        print("S = servo 2 down")
        print("H = home")
        print("Q = quit")
        print("")
        print("Current temporary software limits: 0..180 deg.")
        print("We will replace them with the real mechanical limits later.")
        print("")

        old_settings = termios.tcgetattr(sys.stdin)

        try:
            tty.setcbreak(sys.stdin.fileno())
            self.print_status()

            while not rospy.is_shutdown():
                key = sys.stdin.read(1).lower()

                if key == "a":
                    self.pan = self.clamp(
                        self.pan - self.step, self.pan_min, self.pan_max
                    )
                    self.send_pan()

                elif key == "d":
                    self.pan = self.clamp(
                        self.pan + self.step, self.pan_min, self.pan_max
                    )
                    self.send_pan()

                elif key == "w":
                    self.tilt = self.clamp(
                        self.tilt + self.step, self.tilt_min, self.tilt_max
                    )
                    self.send_tilt()

                elif key == "s":
                    self.tilt = self.clamp(
                        self.tilt - self.step, self.tilt_min, self.tilt_max
                    )
                    self.send_tilt()

                elif key == "h":
                    self.home()

                elif key == "q":
                    break

                self.print_status()

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            print("\nCamera arm teleop stopped.")


if __name__ == "__main__":
    rospy.init_node("stip_camera_arm_teleop")
    CameraArmTeleop().run()
