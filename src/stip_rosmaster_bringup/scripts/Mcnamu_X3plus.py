#!/usr/bin/env python
# encoding: utf-8
import sys
import math
import time
import rospy
import random
import threading
from collections import deque
import numpy as np
from math import pi
from time import sleep
from stip_rosmaster_msgs.msg import *
from stip_rosmaster_msgs.srv import *
from Rosmaster_Lib import Rosmaster
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from stip_rosmaster_bringup.cfg import PIDparamConfig
from std_msgs.msg import String, Float32, Int32, Bool
from sensor_msgs.msg import Imu, MagneticField, JointState


class yahboomcar_driver:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        # 弧度转角度
        # Radians turn angle
        self.RA2DE = 180 / pi
        self.car = Rosmaster()

        # ----------------------------------------------------------
        # EARLIEST POSSIBLE STARTUP BLUE
        #
        # Rosmaster() has just opened the board serial connection.
        # Command a dim blue immediately, before the rest of ROS driver
        # initialization. Default brightness target is 10%.
        #
        # We cannot use set_all_leds() yet because the status parameters and
        # locks have not been created. So send this one direct command here.
        # The status thread will take over later.
        # ----------------------------------------------------------
        try:
            self.car.set_colorful_effect(
                0,
                6,
                parm=1
            )
            sleep(0.01)

            self.car.set_colorful_lamps(
                255,
                0,
                0,
                26
            )
        except Exception:
            pass
        self.car.set_car_type(2)
        self.last_update_time = 1
        self.pos = [0, 0, 0, 0]
        self.imu_link = rospy.get_param("~imu_link", "imu_link")
        self.Prefix = rospy.get_param("~prefix", "")
        self.xlinear_limit = rospy.get_param('~xlinear_speed_limit', 1.0)
        self.ylinear_limit = rospy.get_param('~ylinear_speed_limit', 1.0)
        self.angular_limit = rospy.get_param('~angular_speed_limit', 5.0)

        # ----------------------------------------------------------
        # Bringup status LED + IMU health monitor
        #
        # STARTING : blinking BLUE
        # READY    : solid PURPLE
        # ERROR    : blinking RED
        #
        # All LED writes use THIS driver's existing Rosmaster object,
        # so no second process opens the robot serial port.
        # ----------------------------------------------------------
        self.status_led_enabled = rospy.get_param('~status_led_enabled', True)
        self.status_led_all_id = int(rospy.get_param('~status_led_all_id', 255))
        # Global status LED brightness, 0.0 = off, 1.0 = full brightness.
        self.status_led_brightness = float(
            rospy.get_param('~status_led_brightness', 0.10)
        )
        self.status_led_brightness = max(
            0.0,
            min(1.0, self.status_led_brightness)
        )

        self.status_startup_grace = float(
            rospy.get_param('~status_startup_grace', 5.0)
        )
        self.imu_missing_timeout = float(
            rospy.get_param('~imu_missing_timeout', 1.0)
        )
        self.imu_ready_hold = float(
            rospy.get_param('~imu_ready_hold', 1.0)
        )
        self.imu_freefall_hold = float(
            rospy.get_param('~imu_freefall_hold', 0.5)
        )

        # Auto-scale free-fall detection:
        # learn normal gravity magnitude first, then consider acceleration
        # below this ratio to be free-fall / invalid.
        self.imu_zero_threshold = float(
            rospy.get_param('~imu_zero_threshold', 0.05)
        )
        self.imu_freefall_ratio = float(
            rospy.get_param('~imu_freefall_ratio', 0.20)
        )
        self.imu_baseline_samples_required = int(
            rospy.get_param('~imu_baseline_samples', 15)
        )

        self.status_lock = threading.Lock()
        self.led_lock = threading.Lock()
        self.status_stop_event = threading.Event()

        # Staged startup has TWO distinct readiness meanings:
        #   /bringup_status READY: motors/IMU/odometry health passed
        #   /stip/startup/all_ready True: UI + bringup + camera + QR + fire
        # The former still gates motion; only the latter permits PURPLE.
        # Standalone legacy bringup retains its original READY LED behavior.
        self.status_led_require_all_modules_ready = bool(rospy.get_param(
            '~status_led_require_all_modules_ready', False))
        self._startup_led_lock = threading.Lock()
        self._all_modules_ready = False
        self._stage_heartbeat_at = 0.0
        self._all_ready_received_at = 0.0

        self.status_started_at = time.time()
        self.last_imu_sample_time = None
        self.imu_good_since = None
        self.imu_low_since = None
        self.imu_gravity_baseline = None
        self.imu_baseline_samples = deque(
            maxlen=max(10, self.imu_baseline_samples_required * 2)
        )

        self.system_status = "STARTING"
        self.system_status_reason = "Waiting for valid IMU data"
        self._last_led_rgb = None
        self._last_status_publish = None

        self.statusPublisher = rospy.Publisher(
            'bringup_status',
            String,
            queue_size=10,
            latch=True
        )

        if self.status_led_require_all_modules_ready:
            # No extra serial client! Only this existing driver controls LEDs.
            self._all_ready_sub = rospy.Subscriber(
                '/stip/startup/all_ready', Bool,
                self._all_ready_callback, queue_size=1)
            self._startup_heartbeat_sub = rospy.Subscriber(
                '/stip/startup/heartbeat', Bool,
                self._startup_heartbeat_callback, queue_size=1)
            rospy.loginfo('Status LED PURPLE gated by ALL staged modules READY')

        # Normal status worker starts as soon as its required state exists.
        self.status_thread = threading.Thread(
            target=self.status_led_worker
        )
        self.status_thread.daemon = True
        self.status_thread.start()

        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback, queue_size=100)
        self.sub_RGBLight = rospy.Subscriber("RGBLight", Int32, self.RGBLightcallback, queue_size=100)
        self.sub_Buzzer = rospy.Subscriber("Buzzer", Bool, self.Buzzercallback, queue_size=100)
        self.sub_Arm = rospy.Subscriber("TargetAngle", ArmJoint, self.Armcallback, queue_size=1000)
        self.ArmPubUpdate = rospy.Publisher("ArmAngleUpdate", ArmJoint, queue_size=1000)
        self.EdiPublisher = rospy.Publisher('edition', Float32, queue_size=100)
        self.volPublisher = rospy.Publisher('voltage', Float32, queue_size=100)
        self.staPublisher = rospy.Publisher('joint_states', JointState, queue_size=100)
        self.velPublisher = rospy.Publisher("/pub_vel", Twist, queue_size=100)
        self.imuPublisher = rospy.Publisher("/pub_imu", Imu, queue_size=100)
        self.magPublisher = rospy.Publisher("/pub_mag", MagneticField, queue_size=100)
        self.srv_armAngle = rospy.Service("CurrentAngle", RobotArmArray, self.srv_Armcallback)
        self.dyn_server = Server(PIDparamConfig, self.dynamic_reconfigure_callback)
        self.car.create_receive_threading()
        self.car.set_car_motion(0, 0, 0)
        #self.joints = [90, 2.0, 60.0, 40.0, 90]
        self.joints = [90, 0, 0, 0, 90, 30] #90, 90, 0, 45, 90, 30
        self.car.set_uart_servo_angle_array(self.joints, 1000)

        '''sleep(5)
        self.joints = [90, 2.0, 60.0, 40.0, 90]
        self.car.set_uart_servo_angle_array(self.joints, 1000) '''       

    def set_all_leds(self, red, green, blue):
        """Set the whole RGB bar to one solid RGB color."""
        if not self.status_led_enabled:
            return

        brightness = self.status_led_brightness

        rgb = (
            int(max(0, min(255, red * brightness))),
            int(max(0, min(255, green * brightness))),
            int(max(0, min(255, blue * brightness)))
        )

        # Avoid sending the exact same serial LED command repeatedly.
        if rgb == self._last_led_rgb:
            return

        with self.led_lock:
            try:
                # Stop Yahboom built-in effects first.
                self.car.set_colorful_effect(0, 6, parm=1)
                sleep(0.01)

                # 0xFF / 255 controls the whole programmable light bar.
                self.car.set_colorful_lamps(
                    self.status_led_all_id,
                    rgb[0],
                    rgb[1],
                    rgb[2]
                )

                self._last_led_rgb = rgb

            except Exception as exc:
                rospy.logwarn_throttle(
                    5.0,
                    "Status LED command failed: %s" % str(exc)
                )

    def publish_system_status(self, status, reason):
        message = "%s: %s" % (status, reason)

        if message != self._last_status_publish:
            msg = String()
            msg.data = message
            self.statusPublisher.publish(msg)
            self._last_status_publish = message

            if status == "ERROR":
                rospy.logwarn("Bringup status -> %s" % message)
            else:
                rospy.loginfo("Bringup status -> %s" % message)

    def set_system_status(self, status, reason):
        with self.status_lock:
            changed = (
                status != self.system_status
                or reason != self.system_status_reason
            )

            self.system_status = status
            self.system_status_reason = reason

        if changed:
            self.publish_system_status(
                status,
                reason
            )

    def update_imu_health(self, ax, ay, az):
        """
        Track IMU health without assuming whether Rosmaster acceleration
        is expressed as ~1 g or ~9.81 m/s^2.

        We learn the normal gravity magnitude from valid startup samples.
        Then free-fall/invalid is detected relative to that baseline.
        """
        now = time.time()

        try:
            values = [
                float(ax),
                float(ay),
                float(az)
            ]

            if not all(
                math.isfinite(value)
                for value in values
            ):
                return

            magnitude = math.sqrt(
                values[0] * values[0]
                + values[1] * values[1]
                + values[2] * values[2]
            )

        except Exception:
            return

        with self.status_lock:
            self.last_imu_sample_time = now

            # Exact/near-zero samples are invalid immediately for baseline use.
            if magnitude <= self.imu_zero_threshold:
                if self.imu_low_since is None:
                    self.imu_low_since = now

                self.imu_good_since = None
                return

            # Build an automatic normal-gravity baseline.
            if self.imu_gravity_baseline is None:
                self.imu_baseline_samples.append(
                    magnitude
                )

                if (
                    len(self.imu_baseline_samples)
                    >= self.imu_baseline_samples_required
                ):
                    samples = sorted(
                        list(self.imu_baseline_samples)
                    )
                    middle = len(samples) // 2

                    if len(samples) % 2:
                        baseline = samples[middle]
                    else:
                        baseline = (
                            samples[middle - 1]
                            + samples[middle]
                        ) / 2.0

                    self.imu_gravity_baseline = max(
                        baseline,
                        self.imu_zero_threshold
                    )

                    rospy.loginfo(
                        "IMU gravity baseline learned: %.4f"
                        % self.imu_gravity_baseline
                    )

            # Once baseline is available, use a relative free-fall threshold.
            if self.imu_gravity_baseline is not None:
                freefall_threshold = max(
                    self.imu_zero_threshold,
                    self.imu_gravity_baseline
                    * self.imu_freefall_ratio
                )

                if magnitude < freefall_threshold:
                    if self.imu_low_since is None:
                        self.imu_low_since = now

                    self.imu_good_since = None
                    return

            # Valid IMU sample.
            self.imu_low_since = None

            if self.imu_good_since is None:
                self.imu_good_since = now

    def mark_imu_read_failed(self):
        # Do not fake a timestamp. The watchdog will enter IMU_MISSING
        # once the last real sample is older than imu_missing_timeout.
        with self.status_lock:
            self.imu_good_since = None

    def evaluate_system_status(self):
        now = time.time()

        with self.status_lock:
            last_imu = self.last_imu_sample_time
            good_since = self.imu_good_since
            low_since = self.imu_low_since
            baseline = self.imu_gravity_baseline

        # No IMU sample at all yet.
        if last_imu is None:
            if (
                now - self.status_started_at
                > self.status_startup_grace
            ):
                return (
                    "ERROR",
                    "IMU data missing"
                )

            return (
                "STARTING",
                "Waiting for IMU data"
            )

        # IMU stream stopped updating.
        if (
            now - last_imu
            > self.imu_missing_timeout
        ):
            return (
                "ERROR",
                "IMU data timeout"
            )

        # Near-zero / free-fall-like acceleration persisted long enough.
        if (
            low_since is not None
            and now - low_since >= self.imu_freefall_hold
        ):
            return (
                "ERROR",
                "IMU free-fall / invalid acceleration"
            )

        # Need enough real samples to establish a gravity baseline.
        if baseline is None:
            return (
                "STARTING",
                "Learning IMU gravity baseline"
            )

        # Require stable valid IMU for a short period before READY.
        if (
            good_since is not None
            and now - good_since >= self.imu_ready_hold
        ):
            return (
                "READY",
                "IMU healthy"
            )

        return (
            "STARTING",
            "Waiting for stable IMU"
        )

    def _all_ready_callback(self, msg):
        with self._startup_led_lock:
            self._all_modules_ready = bool(msg.data)
            self._all_ready_received_at = time.monotonic()

    def _startup_heartbeat_callback(self, msg):
        if msg.data:
            with self._startup_led_lock:
                self._stage_heartbeat_at = time.monotonic()

    def _system_led_can_show_purple(self):
        if not self.status_led_require_all_modules_ready:
            return True
        with self._startup_led_lock:
            now = time.monotonic()
            return (self._all_modules_ready and
                    self._all_ready_received_at > 0.0 and
                    now - self._all_ready_received_at < 4.0 and
                    self._stage_heartbeat_at > 0.0 and
                    now - self._stage_heartbeat_at < 3.0)

    def status_led_worker(self):
        """
        STARTING -> blinking blue
        READY    -> purple ONLY when staged all_ready AND heartbeat fresh
        ERROR    -> blinking red
        """
        # Blue is already turned ON during __init__. Keep it on for the
        # first half-cycle, then begin normal blinking.
        blink_on = True
        last_toggle = time.time()

        # Publish initial state immediately.
        self.publish_system_status(
            "STARTING",
            "Waiting for valid IMU data"
        )

        while (
            not rospy.is_shutdown()
            and not self.status_stop_event.is_set()
        ):
            status, reason = self.evaluate_system_status()
            self.set_system_status(
                status,
                reason
            )

            now = time.time()

            if status == "READY" and self._system_led_can_show_purple():
                # Staged mode: all five modules passed AND supervisor is alive.
                # Other launch modes: preserves previous bringup-only behavior.
                self.set_all_leds(160, 0, 255)

            elif status == "ERROR":
                # Blink red.
                if now - last_toggle >= 0.35:
                    blink_on = not blink_on
                    last_toggle = now

                    if blink_on:
                        self.set_all_leds(
                            255,
                            0,
                            0
                        )
                    else:
                        self.set_all_leds(
                            0,
                            0,
                            0
                        )

            else:
                # STARTING, or bringup READY but camera/QR/fire/UI not all READY:
                # blink BLUE. If a module fails while PURPLE, change immediately.
                if self._last_led_rgb == (
                        int(160 * self.status_led_brightness),
                        0,
                        int(255 * self.status_led_brightness)):
                    blink_on = True
                    last_toggle = now
                    self.set_all_leds(0, 0, 255)
                if now - last_toggle >= 0.35:
                    blink_on = not blink_on
                    last_toggle = now

                    if blink_on:
                        self.set_all_leds(
                            0,
                            0,
                            255
                        )
                    else:
                        self.set_all_leds(
                            0,
                            0,
                            0
                        )

            sleep(0.05)

    def force_error_led(self, reason):
        self.set_system_status(
            "ERROR",
            reason
        )

        # Immediate solid red fallback before the normal status thread blink.
        self._last_led_rgb = None
        self.set_all_leds(
            255,
            0,
            0
        )

    def pub_data(self):
        # 发布小车运动速度、陀螺仪数据、电池电压
        ## Publish the speed of the car, gyroscope data, and battery voltage
        while not rospy.is_shutdown():
            sleep(0.05)
            imu = Imu()
            twist = Twist()
            battery = Float32()
            edition = Float32()
            mag = MagneticField()
            try:
                edition.data = self.car.get_version()
                battery.data = self.car.get_battery_voltage()

                ax, ay, az = self.car.get_accelerometer_data()
                gx, gy, gz = self.car.get_gyroscope_data()
                mx, my, mz = self.car.get_magnetometer_data()
                vx, vy, angular = self.car.get_motion_data()

                self.update_imu_health(
                    ax,
                    ay,
                    az
                )

            except Exception as exc:
                self.mark_imu_read_failed()

                rospy.logwarn_throttle(
                    2.0,
                    "Rosmaster sensor read failed: %s"
                    % str(exc)
                )

                continue
            # 发布陀螺仪的数据
            # Publish gyroscope data
            imu.header.stamp = rospy.Time.now()
            imu.header.frame_id = self.imu_link
            imu.linear_acceleration.x = ax
            imu.linear_acceleration.y = ay
            imu.linear_acceleration.z = az
            imu.angular_velocity.x = gx
            imu.angular_velocity.y = gy
            imu.angular_velocity.z = gz
            mag.header.stamp = rospy.Time.now()
            mag.header.frame_id = self.imu_link
            mag.magnetic_field.x = mx
            mag.magnetic_field.y = my
            mag.magnetic_field.z = mz
            # 将小车当前的线速度和角速度发布出去
            # Publish the current linear vel and angular vel of the car
            twist.linear.x = vx
            twist.linear.y = vy
            twist.angular.z = angular
            self.velPublisher.publish(twist)
            # print("ax: %.5f, ay: %.5f, az: %.5f" % (ax, ay, az))
            # print("gx: %.5f, gy: %.5f, gz: %.5f" % (gx, gy, gz))
            # print("mx: %.5f, my: %.5f, mz: %.5f" % (mx, my, mz))
            # rospy.loginfo("battery: {}".format(battery))
            # rospy.loginfo("vx: {}, vy: {}, angular: {}".format(twist.linear.x, twist.linear.y, twist.angular.z))
            self.imuPublisher.publish(imu)
            self.magPublisher.publish(mag)
            self.volPublisher.publish(battery)
            self.EdiPublisher.publish(edition)
            self.joints_states_update()

    def Armcallback(self, msg):
        if not isinstance(msg, ArmJoint): return
        arm_joint = ArmJoint()
        if len(msg.joints) != 0:
            #arm_joint.joints = self.joints
            for i in range(2): 
                self.car.set_uart_servo_angle_array(msg.joints, msg.run_time)
                self.joints = list(msg.joints)
                self.ArmPubUpdate.publish(arm_joint)
                sleep(0.01)
        else:
            arm_joint.id = msg.id
            arm_joint.angle = msg.angle
            for i in range(2):
                self.car.set_uart_servo_angle(msg.id, msg.angle, msg.run_time)
                self.joints[msg.id - 1] = msg.angle
                self.ArmPubUpdate.publish(arm_joint)
                sleep(0.01)
        self.joints_states_update()
        # rospy.loginfo("id: {},angle: {},joints: {},run_time: {}".format(msg.id, msg.angle, msg.joints, msg.run_time))
        sleep(0.001)

    def srv_Armcallback(self, request):
        # 服务端，机械臂当前关节角度
        # Server, the current joint angle of the robotic arm
        if not isinstance(request, RobotArmArrayRequest): return
        # print ("request: ",request)
        response = RobotArmArrayResponse()
        joints = self.car.get_uart_servo_angle_array()
        response.angles = joints
        # print ("Arm joints: ", joints)
        return response

    def RGBLightcallback(self, msg):
        # 流水灯控制，服务端回调函数 RGBLight control
        '''
        effect=[0, 6]，0：停止灯效，1：流水灯，2：跑马灯，3：呼吸灯，4：渐变灯，5：星光点点，6：电量显示
        speed=[1, 10]，数值越小速度变化越快。
        '''
        if not isinstance(msg, Int32): return
        # print ("RGBLight: ", msg.data)
        for i in range(3):
            self.car.set_colorful_effect(msg.data, 6, parm=1)
            sleep(0.01)

    def Buzzercallback(self, msg):
        # 蜂鸣器控制  Buzzer control
        if not isinstance(msg, Bool): return
        # print ("Buzzer: ", msg.data)
        if msg.data:
            for i in range(3):
                self.car.set_beep(1)
                sleep(0.01)
        else:
            for i in range(3):
                self.car.set_beep(0)
                sleep(0.01)

    def cmd_vel_callback(self, msg):
        # 小车运动控制，订阅者回调函数
        # Car motion control, subscriber callback function
        if not isinstance(msg, Twist): return
        # 下发线速度和角速度
        # Issue linear vel and angular vel
        vx = msg.linear.x
        vy = 0.0
        #vy = msg.linear.y
        angular = msg.angular.z
        # 小车运动控制,vel: ±1, angular: ±5
        # Trolley motion control,vel=[-1, 1], angular=[-5, 5]
        # rospy.loginfo("cmd_velx: {}, cmd_vely: {}, cmd_ang: {}".format(vx, vy, angular))
        self.car.set_car_motion(vx, vy, angular)

    def cancel(self):
        """
        ROS shutdown callback.

        Important:
        Do NOT manually unregister publishers/subscribers here. pub_data()
        may still be finishing its current loop iteration while on_shutdown()
        is running. Unregistering first can cause:
            publish() to an unregistered() handle

        rospy will clean up publishers/subscribers automatically when the
        process exits.
        """
        self.status_stop_event.set()

        try:
            self.car.set_car_motion(
                0,
                0,
                0
            )
        except Exception:
            pass

        # Force LEDs OFF on normal shutdown.
        try:
            with self.led_lock:
                self.car.set_colorful_effect(
                    0,
                    6,
                    parm=1
                )
                sleep(0.01)

                self.car.set_colorful_lamps(
                    self.status_led_all_id,
                    0,
                    0,
                    0
                )

                self._last_led_rgb = (
                    0,
                    0,
                    0
                )

        except Exception as exc:
            rospy.logwarn(
                "Could not turn status LED off during shutdown: %s"
                % str(exc)
            )

        rospy.loginfo(
            "Close the robot..."
        )

    def joints_states_update(self):
        state = JointState()
        state.header.stamp = rospy.Time.now()
        state.header.frame_id = "joint_states"
        if len(self.Prefix) == 0:
            state.name = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5", "grip_joint"]
        else:
            state.name = [self.Prefix + "/arm_joint1", self.Prefix + "/arm_joint2",
                          self.Prefix + "/arm_joint3", self.Prefix + "/arm_joint4",
                          self.Prefix + "/arm_joint5", self.Prefix + "/grip_joint"]
        joints = self.joints[:]
        joints[5] = np.interp(joints[5], [30, 180], [0, 90])
        mid = np.array([90, 90, 90, 90, 90, 90])
        array = np.array(np.array(joints) - mid)
        DEG2RAD = np.array([pi / 180])
        position_src = list(np.dot(array.reshape(-1, 1), DEG2RAD))
        state.position = position_src
        self.staPublisher.publish(state)

    def dynamic_reconfigure_callback(self, config, level):
        # self.car.set_pid_param(config['Kp'], config['Ki'], config['Kd'])
        # print("PID: ", config['Kp'], config['Ki'], config['Kd'])
        self.linear_max = config['linear_max']
        self.linear_min = config['linear_min']
        self.angular_max = config['angular_max']
        self.angular_min = config['angular_min']
        if config['SetArmjoint']:
            self.car.set_uart_servo_angle_array(
                [config['joint1'], config['joint2'], config['joint3'],
                 config['joint4'], config['joint5'], config['joint6']], run_time=1000)
        return config

if __name__ == '__main__':
    rospy.init_node(
        "driver_node",
        anonymous=False
    )

    driver = None

    try:
        driver = yahboomcar_driver()

        sleep(2)

        #driver.joints = [90, 2.0, 60.0, 40.0, 90]
        #driver.car.set_uart_servo_angle_array(driver.joints, 1000)

        driver.pub_data()
        rospy.spin()

    except rospy.ROSInterruptException:
        # Normal roslaunch / Ctrl-C shutdown.
        pass

    except Exception as exc:
        # If ROS is already shutting down, treat any late publish/read
        # exception as part of normal shutdown, NOT as a system fault.
        if rospy.is_shutdown():
            rospy.loginfo(
                "Driver stopped during ROS shutdown: %s"
                % str(exc)
            )

        else:
            rospy.logerr(
                "Driver exception: %s"
                % str(exc)
            )

            if driver is not None:
                try:
                    driver.force_error_led(
                        "Driver exception"
                    )
                except Exception:
                    pass

