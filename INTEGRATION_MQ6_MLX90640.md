# STIP v16 — POP32i MQ-6 and MLX90640 Thermal Camera

**Target:** ROS1 Noetic on Jetson Orin NX (JetPack 5.x); POP32i + DT-SENSE MQ-6 UART; MLX90640 on a verified Linux I2C bus.

## Architecture and module restart policy

- `stip_gas_sensor`: uses *only* the POP32i USB-C **COM** serial connection at 115200 baud. POP32i talks to DT-SENSE MQ-6 on a **different** 38400-baud UART, sends raw ADC, receives buzzer commands. Calibration runs on Jetson and stores a baseline in `~/.ros/stip_mq6_baseline.json`.
- `stip_thermal_camera`: reads a real 32×24 Celsius MLX90640 grid over `/dev/i2c-0` by default (device address `0x33`), publishes **real measured temperature** and an independently interpolated 320×240 visualization.
- `stip_rosmaster_manager`: QR, fire, gas and thermal each get **3 failed start attempts** per recovery cycle (first attempt + 2 retries, with configurable 5-s spacing). Then `BLOCKED`: no automatic attempts until `restart qr|fire|gas|thermal`. Bringup remains manual-only; UI/camera retain v15 recovery behavior. Gas and thermal recovery are each supervised in separate workers, and never gate robot `all_ready` / purple LED. A live node must also produce valid sensor health data to be considered READY.
- No new sensor is started by unrelated standalone launch files. `system.launch` starts/supervises modules together.

### Before starting: required packages on Jetson

```bash
sudo apt update
sudo apt install -y i2c-tools python3-serial python3-numpy python3-opencv python3-smbus ros-noetic-cv-bridge
python3 -m pip install --user adafruit-circuitpython-mlx90640 adafruit-extended-bus
python3 -c 'import adafruit_mlx90640, adafruit_extended_bus, serial, cv2, numpy, cv_bridge; print("imports OK")'
```

The installed pip packages must be visible to **the same Python 3 environment** used by ROS Noetic. If Jetson reports `Permission denied: '/dev/i2c-0'`, inspect `ls -l /dev/i2c-0` and group membership. Add the logged-in user to the existing `i2c` group when provided by your distribution (`sudo usermod -aG i2c "$USER"`; log out and back in). Do not run all ROS nodes as root or set world-write permissions on the bus as a shortcut.

### Confirm the thermal device address and physical bus

**Important:** Jetson connector labeling and Linux `/dev/i2c-N` enumeration are not necessarily the same. `i2c_bus:=0` selects literal `/dev/i2c-0`, not necessarily the connector printed `I2C0`. Confirm your own board/carrier pin mapping and device address before trying acquisition.

```bash
ls /dev/i2c-*
i2cdetect -l
sudo i2cdetect -y 0 0x33 0x33
```

The address `33` in the output means the MLX90640 acknowledged at **0x33**. `--` means the device did not respond; `UU` typically means a kernel driver owns that address. Limit the probe to **0x33**, not the entire Jetson I2C bus; on-board controllers may occupy other addresses. Do not attach 5V I2C signals to Jetson 3.3V pins. Verify MLX90640 breakout VCC range and wiring (SDA↔SDA, SCL↔SCL, GND↔GND, appropriate 3.3V power) for your *particular* board.

For initial reliability use 2 Hz, then try 4 Hz only after testing stability. Data reads require many I2C transfers; I2C throughput and clock configuration limit practical frame rate. The module opens the selected bus directly with Adafruit ExtendedI2C, rather than silently using Jetson's default bus.

### Confirm POP32i COM port

Connect POP32i's **COM** USB port to the Jetson (its SWD/DEBUG port is solely for uploading firmware). Identify the correct port; don't accidentally select the ROSMASTER motor controller's serial device:

```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
udevadm info -q property -n /dev/ttyACM0 | grep -E 'ID_SERIAL|ID_VENDOR|ID_MODEL'
```

Prefer a stable `/dev/serial/by-id/...` path when present. Otherwise use the verified `/dev/ttyACM0` or `/dev/ttyUSB*`. USB Serial must not be opened simultaneously by Arduino Serial Monitor and the ROS node.

### Standalone tests (do not run over manager launch)

```bash
cd ~/Documents/stip_rosmaster_ws
./build_local.sh
source devel/setup.bash --extend

# Terminal 1 — gas ONLY, no robot bringup or other sensors
roslaunch stip_gas_sensor gas_sensor.launch port:=/dev/ttyACM0
# Terminal 2
rostopic echo /gas_sensor/raw
rostopic echo -n 1 /gas_sensor/status

# Test buzzer (explicitly off again)
rostopic pub -1 /gas_sensor/beep_ms std_msgs/UInt16 'data: 200'
rostopic pub -1 /gas_sensor/alarm_cmd std_msgs/Bool 'data: true'
rostopic pub -1 /gas_sensor/alarm_cmd std_msgs/Bool 'data: false'

# Start Jetson-side baseline calibration ONLY IN CLEAN AIR
rosservice call /gas_sensor/calibrate '{}'
rostopic echo -n 1 /gas_sensor/calibration_status
rostopic echo -n 1 /gas_sensor/baseline
rostopic echo /gas_sensor/change_percent
```

The calibration service starts collection asynchronously: 60 successful valid sensor samples at roughly 2 Hz, followed by a basic spread/stability test, and stores the baseline. Use it **only after manufacturer-recommended initial sensor conditioning**, away from suspect gas sources. It is **not a ppm / %LEL calibration**, a certified gas detector, or proof that an environment is safe.

```bash
# Independent thermal camera test with local OpenCV viewer (GUI DISPLAY needed)
roslaunch stip_thermal_camera thermal_test.launch i2c_bus:=0
# In another terminal
rostopic echo /thermal/max_c
rostopic hz /thermal/image_color
```

For headless testing:

```bash
roslaunch stip_thermal_camera thermal_camera.launch i2c_bus:=0
rostopic echo -n 1 /thermal/status
```

If started under the manager already, do **not** launch the thermal camera again. Instead, to view the same stream:

```bash
rosrun stip_thermal_camera thermal_viewer.py
```

### Managed whole-system launch

```bash
roslaunch stip_rosmaster_manager system.launch gas_serial_port:=/dev/ttyACM0 thermal_i2c_bus:=0
```

In a separate sourced terminal:

```bash
bash ~/Documents/stip_rosmaster_ws/src/stip_rosmaster_manager/tools/stip_stage.sh status
bash ~/Documents/stip_rosmaster_ws/src/stip_rosmaster_manager/tools/stip_stage.sh restart gas
bash ~/Documents/stip_rosmaster_ws/src/stip_rosmaster_manager/tools/stip_stage.sh restart thermal
bash ~/Documents/stip_rosmaster_ws/src/stip_rosmaster_manager/tools/stip_stage.sh restart qr
bash ~/Documents/stip_rosmaster_ws/src/stip_rosmaster_manager/tools/stip_stage.sh restart fire
```

`restart` queues one restart and resets that stage's three-attempt budget; a physical fault remains a fault and will cause `BLOCKED` again after three failed starts.

### Topics and services

| Interface | Type | Description |
|---|---|---|
| `/gas_sensor/raw` | `std_msgs/Int32` | Latest valid 0..1023 MQ-6 ADC |
| `/gas_sensor/healthy` | `std_msgs/Bool` | Valid recent sensor communication |
| `/gas_sensor/connected` | `std_msgs/Bool` | Valid POP32i protocol data being received |
| `/gas_sensor/status` | `std_msgs/String` | STARTING, OK, SENSOR_ERROR, DATA_TIMEOUT |
| `/gas_sensor/baseline` | `std_msgs/Int32` | Saved/obtained clean-air ADC baseline |
| `/gas_sensor/change_percent` | `std_msgs/Float32` | Relative ADC change (can be negative); NOT %LEL |
| `/gas_sensor/calibration_status` | `std_msgs/String` | Clean-air calibration progress/result |
| `/gas_sensor/alarm_cmd` | `std_msgs/Bool` input | Jetson commands POP32i buzzer ON/OFF |
| `/gas_sensor/beep_ms` | `std_msgs/UInt16` input | One beep (50..2000 milliseconds) |
| `/gas_sensor/alarm_state` | `std_msgs/Bool` | POP32i alarm ACK state |
| `/gas_sensor/calibrate` | `std_srvs/Trigger` | Start clean-air ADC baseline collection |
| `/thermal/image_raw` | `sensor_msgs/Image` 32FC1 | Measured 32×24 temperatures in Celsius |
| `/thermal/temperatures` | `std_msgs/Float32MultiArray` | Same measured 768 native Celsius values |
| `/thermal/image_color` | `sensor_msgs/Image` bgr8 | 320×240 cubic-interpolated false-color **preview** |
| `/thermal/min_c` | `std_msgs/Float32` | Minimum native-frame temperature |
| `/thermal/max_c` | `std_msgs/Float32` | Maximum native-frame temperature |
| `/thermal/center_c` | `std_msgs/Float32` | Center native pixel temperature |
| `/thermal/healthy` | `std_msgs/Bool` | Valid frame received |
| `/thermal/status` | `std_msgs/String` | Sensor status/errors |
| `/stip/startup/modules` | `std_msgs/String` (JSON) | All seven managed stages and their states |
| `/stip/modules/restart_gas` | `std_srvs/Trigger` | Manual restart after failure |
| `/stip/modules/restart_thermal` | `std_srvs/Trigger` | Manual restart after failure |

### Safety and limitations

- Buzzer only sounds when a Jetson command is sent; **no automatic calibrated gas alarm** is implemented. POP32i keeps an already asserted repeating alarm ON if the Jetson disconnects, until a received `ALARM,OFF` command or a board reset. Do not use this prototype as a protective gas warning system.
- MQ-6 is one broad-response gas element. Raw ADC and clean-air baseline percentage are not ppm or %LEL; proper span calibration needs a known LPG concentration and a certified instrument.
- MLX90640 is **32×24 physical pixels**. Bicubic interpolation improves appearance but not spatial accuracy or measurement resolution. Color scale defaults to fixed **15–45°C** for comparable views; change `config/thermal.yaml` to tune or use auto-ranging for high-contrast viewing.
- MLX90640 temperature estimates depend on surface emissivity and reflected ambient radiation; don't infer safety-critical temperature thresholds without verification.
- Full system is untested on the actual Jetson / POP32i during packaging. First verify ROS dependencies, GPIO/USB wiring, bus mapping, and an individual launch for each sensor.
