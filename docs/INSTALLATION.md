# Jetson setup and installation

This release targets the **existing** NVIDIA Jetson Orin NX / JetPack 5.x (Ubuntu 20.04) / ROS Noetic stack, not a fresh generic Ubuntu/desktop ROS installation. The project depends on the Yahboom robot/vendor software and a CUDA/TensorRT-compatible deployment.

## 1. Verify the target machine

```bash
uname -m                       # expected: aarch64
lsb_release -a
printenv ROS_DISTRO            # expected: noetic after sourcing ROS
ls /opt/ros/noetic/setup.bash
ls ~/software/library_ws/devel/setup.bash
nvidia-smi || true             # may be absent on Jetson
```

**Before editing anything:** back up the current Jetson workspace and the external inspection data/configuration. Never overwrite a working `~/Documents/stip_rosmaster_ws` with a Git checkout without first comparing its changes.

## 2. System and ROS dependencies

Expected ROS packages include `roscpp`, `rospy`, `tf`, `tf2_ros`, `robot_localization`, `navigation` (`move_base`, `amcl`, `map_server`), `gmapping`, `dynamic_reconfigure`, `cv_bridge`, `rqt_reconfigure`, and `ydlidar_ros_driver` from the vendor workspace. Hardware may also require access to USB/serial and I²C. The original launch files are the authority for actual dependencies.

Inspect individual packages:

```bash
source /opt/ros/noetic/setup.bash
source ~/software/library_ws/devel/setup.bash --extend  # if installed
rospack find ydlidar_ros_driver
python3 -c 'import Rosmaster_Lib; print("Yahboom library available")'
```

ROS Noetic Python 3.8 libraries used by sensor/inspection code include `numpy`, `opencv-python` as provided by the validated Jetson stack, `pyzbar`/ZBar, `pyserial`, and the Adafruit MLX90640/Extended Bus drivers. The gas/thermal integration guide provides specific installation and bus checks:

[`../INTEGRATION_MQ6_MLX90640.md`](../INTEGRATION_MQ6_MLX90640.md).

The **native fire detector** additionally requires JetPack-matched CUDA 11.4, TensorRT (8.5 used in the source comments), OpenCV, and GStreamer development packages. `src/stip_fire_detection/CMakeLists.txt` compiles a CUDA/TensorRT binary for `sm_87`. Avoid untested global upgrades of CUDA/TensorRT/Jetson Torch/Ultralytics; this repository deliberately does not include a universal `pip requirements.txt`.

## 3. Clone and build

```bash
mkdir -p ~/Documents
cd ~/Documents
git clone https://github.com/<YOUR_USERNAME>/stip-rosmaster.git stip_rosmaster_ws
cd stip_rosmaster_ws
./build_local.sh
source devel/setup.bash --extend
./check_local.sh
python3 tools/verify_workspace.py
```

`src/CMakeLists.txt` is the canonical catkin symlink. If an external ZIP extractor flattens it, `./build_local.sh` restores it.

If launch scripts are copied by a filesystem that discards executable bits, run `chmod +x build_local.sh check_local.sh src/stip_rosmaster_manager/scripts/start_dashboard.sh`, then restore executable permissions on `src/stip_*/scripts/*.py` before using `roslaunch`. The GitHub repository and supplied ZIP preserve the Unix modes.

## 4. Confirm hardware interfaces before operating

- Base/motors: verify wiring, power, controller interfaces, encoder feedback, `/odom` and `/cmd_vel`; keep wheels raised on first tests.
- LiDAR: verify the vendor driver and TF frame relationships (`map`, `odom`, `base_link`/`base_footprint`, `laser`).
- Camera: only **one** process should open the same USB camera; the shared ROS stream is `/inspection_camera/image_raw/compressed`.
- Pan/tilt: robot driver receives `stip_rosmaster_msgs/ArmJoint` on `/TargetAngle`; avoid simultaneous manual/QR/fire publishers.
- Thermal: confirm the physical MLX90640 I²C bus (default in current headless launch: `/dev/i2c-1`) and address `0x33`; use the `thermal_rgb_alignment.yaml` appropriate to this exact camera mounting.
- MQ-6: POP32i firmware communicates over USB serial, commonly `/dev/ttyACM0`; verify the port (do not assume it is stable across reboots). The clean-air baseline is *relative ADC*, not ppm.
- Fire AI: confirm the saved TensorRT engine is compatible with the current Jetson, TensorRT version and device.

## 5. Start the chosen interface

**Web dashboard** (default and recommended for this repository):

```bash
cd ~/Documents/stip_rosmaster_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash --extend
./src/stip_rosmaster_manager/scripts/start_dashboard.sh
```

Default LAN access: `http://<JETSON_IP>:8765`. Binding to `0.0.0.0` does **not** mean visitors should type that address. Optional environment variables:

| Variable | Default | Meaning |
| --- | --- | --- |
| `STIP_DASHBOARD_BIND` | `0.0.0.0` | IPv4 listener; set `127.0.0.1` for local-only |
| `STIP_DASHBOARD_PORT` | `8765` | Web port |
| `STIP_DASHBOARD_TOKEN` | unset | Optional browser authentication token |
| `STIP_AUTO_START_SYSTEM` | `1` | Start headless manager only if absent |
| `STIP_NAV_BACKEND` | `stip` | Navigation backend (`stip` or optional `yahboom`) |
| `STIP_WS` | `~/Documents/stip_rosmaster_ws` | Workspace path for the launcher |

**Desktop legacy GUI** requires a graphical Jetson session and should be run **instead of** dashboard-owned stage startup:

```bash
roslaunch stip_rosmaster_manager launcher.launch
```

**Headless ROS manager without the web UI:**

```bash
roslaunch stip_rosmaster_manager dashboard_system.launch
```

## 6. Quick health checks

```bash
bash src/stip_rosmaster_manager/tools/stip_stage.sh status
rostopic hz /inspection_camera/image_raw/compressed
rostopic echo -n 1 /odom
rostopic echo -n 1 /thermal/healthy
rostopic echo -n 1 /gas_sensor/raw
```

Stop a dashboard in its terminal with `Ctrl+C`. Do not kill shared ROS drivers solely because a browser tab is closed. See [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md).
