#!/usr/bin/env python3
"""POP32i/DT-SENSE MQ-6 USB UART bridge. Calibration is Jetson-side only.

Serial POP32i -> host: MQ6,<millis>,<adc|-1>,OK|ERROR
Serial host -> POP32i: PING, ALARM,ON/OFF, BEEP,<50..2000>
No gas concentration or LEL safety claim is made by this node.
"""
import json
import os
import queue
import statistics
import time
from pathlib import Path

import rospy
import serial
from std_msgs.msg import Bool, Float32, Int32, String, UInt16
from std_srvs.srv import Trigger, TriggerResponse


class GasBridge:
    def __init__(self):
        self.port = rospy.get_param('~port', '/dev/ttyACM0')
        self.baud = int(rospy.get_param('~baud', 115200))
        self.timeout = float(rospy.get_param('~sample_timeout_sec', 5.0))
        self.cal_samples = max(10, int(rospy.get_param('~calibration_samples', 60)))
        self.max_spread = max(1, int(rospy.get_param('~calibration_max_spread_adc', 30)))
        self.baseline_path = Path(os.path.expanduser(rospy.get_param('~baseline_file', '~/.ros/stip_mq6_baseline.json')))
        self.baseline = self.load_baseline()
        self.calibration = None
        self.requested_commands = queue.Queue(maxsize=20)
        self.last_sample = time.monotonic()
        self.last_ping = 0.0
        self.last_status = 0.0
        self.connection_seen = False
        self.alarm_on = False

        self.raw_pub = rospy.Publisher('/gas_sensor/raw', Int32, queue_size=10)
        self.health_pub = rospy.Publisher('/gas_sensor/healthy', Bool, queue_size=1)
        self.connected_pub = rospy.Publisher('/gas_sensor/connected', Bool, queue_size=1)
        self.status_pub = rospy.Publisher('/gas_sensor/status', String, queue_size=1, latch=True)
        self.base_pub = rospy.Publisher('/gas_sensor/baseline', Int32, queue_size=1, latch=True)
        self.change_pub = rospy.Publisher('/gas_sensor/change_percent', Float32, queue_size=10)
        self.cal_pub = rospy.Publisher('/gas_sensor/calibration_status', String, queue_size=1, latch=True)
        self.alarm_status_pub = rospy.Publisher('/gas_sensor/alarm_state', Bool, queue_size=1, latch=True)
        rospy.Subscriber('/gas_sensor/alarm_cmd', Bool, self.alarm_callback, queue_size=10)
        rospy.Subscriber('/gas_sensor/beep_ms', UInt16, self.beep_callback, queue_size=10)
        rospy.Service('/gas_sensor/calibrate', Trigger, self.calibrate_callback)
        if self.baseline is not None:
            self.base_pub.publish(Int32(data=self.baseline))
            self.cal_pub.publish(String(data='SAVED_BASELINE (verify in clean air)'))
        else:
            self.cal_pub.publish(String(data='NOT_CALIBRATED'))
        self.alarm_status_pub.publish(Bool(data=False))
        self.status_pub.publish(String(data='STARTING'))
        self.health_pub.publish(Bool(data=False))
        self.connected_pub.publish(Bool(data=False))
        self.ser = None

    def load_baseline(self):
        try:
            info = json.loads(self.baseline_path.read_text())
            value = int(info['baseline_adc'])
            if 1 <= value <= 1023:
                rospy.loginfo('MQ-6 previously saved baseline: %d', value)
                return value
        except (OSError, ValueError, KeyError, TypeError):
            pass
        return None

    def alarm_callback(self, msg):
        self.enqueue('ALARM,ON' if msg.data else 'ALARM,OFF')

    def beep_callback(self, msg):
        duration = int(msg.data)
        if 50 <= duration <= 2000:
            self.enqueue('BEEP,{}'.format(duration))
        else:
            rospy.logwarn('BEEP command must be 50..2000 ms')

    def enqueue(self, command):
        try:
            self.requested_commands.put_nowait(command)
        except queue.Full:
            rospy.logwarn('MQ-6 command queue full: dropped %s', command)

    def calibrate_callback(self, _req):
        if not self.connection_seen or time.monotonic() - self.last_sample > self.timeout:
            return TriggerResponse(False, 'Sensor not reporting valid measurements')
        if self.calibration is not None:
            return TriggerResponse(False, 'Calibration already running')
        # Service runs concurrently with the main read loop; list assignment is atomic.
        self.calibration = []
        self.cal_pub.publish(String(data='COLLECTING'))
        return TriggerResponse(True, 'Collecting {} valid samples; keep sensor in clean stable air'.format(self.cal_samples))

    def transmit(self, command):
        self.ser.write((command + '\n').encode('ascii'))

    def handle_line(self, line):
        if line == 'PONG':
            return
        if line.startswith('ACK,'):
            if line == 'ACK,ALARM,ON':
                self.alarm_on = True
                self.alarm_status_pub.publish(Bool(data=True))
            elif line == 'ACK,ALARM,OFF':
                self.alarm_on = False
                self.alarm_status_pub.publish(Bool(data=False))
            rospy.loginfo('POP32i %s', line)
            return
        if line.startswith('ERR,'):
            rospy.logwarn('POP32i %s', line)
            return
        fields = line.split(',')
        if len(fields) != 4 or fields[0] != 'MQ6':
            if line.startswith('READY,'):
                rospy.loginfo('POP32i boot: %s', line)
            return
        try:
            int(fields[1])  # POP32i uptime, for protocol verification
            raw = int(fields[2])
        except ValueError:
            return
        if fields[3] != 'OK' or not 0 <= raw <= 1023:
            # Tolerate isolated UART errors; if good samples stop for >timeout,
            # the watchdog publishes unhealthy and exits for manager recovery.
            self.status_pub.publish(String(data='SENSOR_ERROR'))
            return

        self.connection_seen = True
        self.last_sample = time.monotonic()
        self.raw_pub.publish(Int32(data=raw))
        self.health_pub.publish(Bool(data=True))
        self.connected_pub.publish(Bool(data=True))
        self.status_pub.publish(String(data='OK'))
        if self.baseline is not None and self.baseline > 0:
            self.change_pub.publish(Float32(data=100.0 * (raw - self.baseline) / self.baseline))

        collecting = self.calibration
        if collecting is not None:
            collecting.append(raw)
            if len(collecting) >= self.cal_samples:
                self.finish_calibration(collecting)

    def finish_calibration(self, values):
        self.calibration = None
        if max(values) - min(values) > self.max_spread:
            self.cal_pub.publish(String(data='FAILED_UNSTABLE'))
            rospy.logwarn('MQ-6 baseline rejected: unstable ADC range %d..%d', min(values), max(values))
            return
        baseline = int(round(statistics.mean(values)))
        if baseline <= 0:
            self.cal_pub.publish(String(data='FAILED_ZERO_BASELINE'))
            return
        self.baseline = baseline
        self.base_pub.publish(Int32(data=baseline))
        self.cal_pub.publish(String(data='CALIBRATED_RELATIVE_ONLY'))
        try:
            self.baseline_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.baseline_path.with_suffix('.tmp')
            temporary.write_text(json.dumps({'baseline_adc': baseline, 'samples': len(values),
                                             'timestamp_unix': int(time.time())}, indent=2) + '\n')
            os.replace(str(temporary), str(self.baseline_path))
            rospy.loginfo('MQ-6 relative clean-air baseline saved: %d', baseline)
        except OSError as exc:
            rospy.logerr('Could not save MQ-6 baseline: %s', exc)

    def run(self):
        rospy.loginfo('Opening POP32i on %s at %d baud', self.port, self.baud)
        self.ser = serial.Serial(self.port, self.baud, timeout=0.10, write_timeout=0.5)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('MQ-6 bridge active; no automatic ppm/%sLEL alarm', '%')
        while not rospy.is_shutdown():
            now = time.monotonic()
            if now - self.last_ping >= 1.0:
                self.transmit('PING')
                self.last_ping = now
            try:
                for _ in range(5):
                    command = self.requested_commands.get_nowait()
                    self.transmit(command)
            except queue.Empty:
                pass
            line = self.ser.readline().decode('ascii', errors='replace').strip()
            if line:
                self.handle_line(line)
            now = time.monotonic()
            if now - self.last_sample > self.timeout:
                self.health_pub.publish(Bool(data=False))
                self.connected_pub.publish(Bool(data=False))
                self.status_pub.publish(String(data='DATA_TIMEOUT'))
                raise RuntimeError('No valid MQ-6 data for {:.1f}s; manager may restart'.format(self.timeout))
            if now - self.last_status >= 1.0:
                self.last_status = now
                self.connected_pub.publish(Bool(data=self.connection_seen))

    def shutdown(self):
        self.health_pub.publish(Bool(data=False))
        self.connected_pub.publish(Bool(data=False))
        if self.ser is not None:
            try:
                # Never silently cancel an asserted gas alarm during recovery.
                self.ser.close()
            except (OSError, serial.SerialException):
                pass


if __name__ == '__main__':
    rospy.init_node('stip_gas_sensor')
    try:
        GasBridge().run()
    except (OSError, ValueError, serial.SerialException, RuntimeError) as error:
        rospy.logerr('MQ-6 bridge error: %s', error)
        raise SystemExit(1)
