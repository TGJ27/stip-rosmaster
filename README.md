# STIP ROSMASTER

**Version 1.0.0 — first GitHub source release (27 October 2026)**  
ROS 1 Noetic · NVIDIA Jetson Orin NX · Yahboom ROSMASTER X3 Plus · Python 3.8 · C++/CUDA

STIP ROSMASTER is a ROS-based robot mission-control system for mapping, localization, waypoint navigation, QR and visual-indicator inspection, RGB/thermal fire monitoring, and relative MQ-6 gas monitoring. It includes a desktop ROS stage manager and a modular web dashboard accessible on the robot's local Wi-Fi/hotspot.

> **Hardware-specific source release.** This is a cleaned snapshot of the operator's current working workspace, **not** a plug-and-play ROS distribution or a hardware-certified safety system. Source/configuration files were retained; only release packaging, documentation, integrity metadata, and one portable map image path were changed. This GitHub export was statically checked outside the Jetson; it was **not** run against physical hardware during release preparation.

## At a glance

| Subsystem | Capabilities |
| --- | --- |
| Robot base | Yahboom motor/IMU integration, EKF, velocity control and calibration helpers |
| Navigation | LiDAR mapping, occupancy map save/load, AMCL localization, `move_base`, manual control and missions |
| Inspection | Compressed MJPEG camera, camera-arm pan/tilt, QR profiles, indicator ROI, test inspection and evidence gallery |
| Safety monitoring | C++ TensorRT fire/smoke AI, MLX90640 thermal sensing, calibrated RGB/thermal overlay and fusion/auto-aim |
| Gas monitoring | POP32i-to-MQ-6 serial bridge, raw ADC / clean-air baseline, manual calibration and alarm commands |
| Operations | Stage supervision, web dashboard, live views, mission editing, patrol control, logs and operator confirmations |

## Supported environment / important requirements

- **Target:** Jetson Orin NX running the **existing validated JetPack 5 / Ubuntu 20.04 / ROS Noetic** installation. The C++ fire package is explicitly built for CUDA 11.4, TensorRT 8.x (developed with 8.5), and `sm_87` Orin GPUs. See [`docs/INSTALLATION.md`](docs/INSTALLATION.md).
- The **Yahboom/vendor workspace** and ROS packages (not bundled) must already provide `Rosmaster_Lib`, `ydlidar_ros_driver`, robot hardware interfaces, and standard ROS navigation packages. The helper scripts look for `~/software/library_ws/devel/setup.bash`.
- The repository includes a hardware-specific `fire_smoke_yolov8s.engine` and ROSMASTER CAD meshes because this snapshot uses them. **Verify that you have permission to redistribute these vendor/model assets before making the repository public.** TensorRT serialized engines are not guaranteed portable across JetPack/TensorRT/GPU versions.
- ROS1 Noetic is an **end-of-life** ROS generation. Keep this robot on a controlled and trusted network.

## 1. Clone / place the workspace

The GitHub repository root is a **catkin workspace**, containing `src/`, `build_local.sh`, and `check_local.sh`.

```bash
mkdir -p ~/Documents
cd ~/Documents
git clone https://github.com/<YOUR_USERNAME>/stip-rosmaster.git stip_rosmaster_ws
cd stip_rosmaster_ws
```

For the downloadable ZIP, extract its `stip_rosmaster_ws/` directory into `~/Documents/` instead. Replace `<YOUR_USERNAME>` after creating your repository.

## 2. Build the workspace

Do this on the **Jetson** after checking the hardware/vendor dependencies. Do not indiscriminately upgrade Jetson PyTorch, CUDA, TensorRT or OpenCV with pip.

```bash
cd ~/Documents/stip_rosmaster_ws
./build_local.sh
source devel/setup.bash --extend
./check_local.sh
python3 tools/verify_workspace.py
```

The build helper loads ROS Noetic and the installed vendor workspace when available. It also repairs `src/CMakeLists.txt` if an archive tool flattened the catkin symlink. For full prerequisites and device checks, see [`docs/INSTALLATION.md`](docs/INSTALLATION.md).

## 3. Start the web dashboard — recommended

With the robot stationary, power and emergency-stop arrangements verified, launch **one** system owner:

```bash
cd ~/Documents/stip_rosmaster_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash --extend
./src/stip_rosmaster_manager/scripts/start_dashboard.sh
```

The launcher listens on **`0.0.0.0:8765`** by default and can start the existing headless stage manager if it is not already running. Other devices on the **same trusted Wi-Fi or Jetson hotspot** access `http://JETSON_IP:8765/` (use `hostname -I` to find its address). You do **not** browse to `0.0.0.0`; it is a listening address.

**Network security:** the default local dashboard has **no mandatory login**. Anyone who can reach the port can potentially move the robot or request host power operations. Use a private WPA2/WPA3-protected network or localhost binding, avoid exposing port 8765 to the internet, and configure an optional token before starting:

```bash
export STIP_DASHBOARD_TOKEN='YOUR_STRONG_RANDOM_TOKEN'
# Optional: use localhost only instead of all network interfaces:
# export STIP_DASHBOARD_BIND=127.0.0.1
./src/stip_rosmaster_manager/scripts/start_dashboard.sh
```

If the ROS manager is already started by another program, the dashboard **attaches** instead of creating another manager. To avoid automatic manager startup entirely, set `STIP_AUTO_START_SYSTEM=0`.

## 4. Basic operator workflow

Use the sidebar: **Home → Maps → Mission → Patrol → Settings**. The recommended sequence is:

1. **Home:** confirm ROS connection, robot bringup, LiDAR/camera/sensor health. Resolve errors before driving.
2. **Maps:** create a named occupancy map, teleoperate to cover the environment, and **Save Map**. Existing map YAML/PGM pairs can be previewed or selected. Changing pages during active mapping requires confirmation.
3. **Mission:** choose a saved map, visually confirm the robot pose (or drag to publish a corrected initial pose), drive to a location, and use **Add Current Pose** to create each waypoint with its yaw. Optionally add QR inspection or pause tasks. Save the mission.
4. **QR enrollment:** use the live RGB camera and arm controls to aim, detect the QR, draw the indicator ROI, and **Save Enrollment**. A saved QR task can be viewed or tested using the existing ROS inspection service; saved evidence is available in the gallery.
5. **Patrol:** load a mission, confirm localization, start/pause/resume/stop; inspect the RGB/thermal views, progress and activity/evidence records. Leaving an active workflow requests cleanup and confirmation.
6. **Settings:** manage fire/smoke confidence, camera tracking parameters (controller restart may be required), MQ-6 clean-air baseline and diagnostics.

**Safety:** UI localization confirmation is an operator decision, not a guarantee of valid AMCL localization. Recheck the actual pose before sending autonomous goals. Test motion first with the wheels raised; do not drive by dashboard and `teleop_twist_keyboard` simultaneously. A software emergency stop does **not** replace a physical stop circuit.

See the detailed [`docs/OPERATOR_GUIDE.md`](docs/OPERATOR_GUIDE.md).

## 5. Standalone ROS entrypoints (advanced / troubleshooting)

Use **one owning workflow at a time**. The following standalone launch files can duplicate hardware drivers if launched beside the dashboard-managed system:

```bash
# Desktop workflow launcher, graphical Jetson session only
roslaunch stip_rosmaster_manager launcher.launch

# Original ROS staged system (instead of the headless dashboard manager)
roslaunch stip_rosmaster_manager system.launch

# Complete mapping or navigation, standalone alternatives
roslaunch stip_rosmaster_nav mapping_full.launch
roslaunch stip_rosmaster_nav navigation_full.launch map:=my_map

# Independent camera / QR / arm / thermal / gas tests
roslaunch stip_inspection_camera camera_test.launch
roslaunch stip_qr_inspection inspection.launch
roslaunch stip_camera_arm teleop.launch
roslaunch stip_thermal_camera thermal_test.launch
roslaunch stip_gas_sensor gas_sensor.launch

# Standalone fire/thermal fusion (attended, stationary testing only)
roslaunch stip_fire_inspection fire_inspection_live.launch
```

The last entrypoint can start several dependencies and is **not** a safe second instance alongside an active patrol. Further launch parameters and dependencies: [`docs/LAUNCH_CATALOG.md`](docs/LAUNCH_CATALOG.md).

## Project layout

```text
stip_rosmaster_ws/
├── src/                  # 12 ROS packages (see docs/ARCHITECTURE.md)
├── firmware/             # POP32i Arduino sketch for the MQ-6 bridge
├── tools/verify_workspace.py
├── docs/                 # Installation, operator, launch and release documentation
├── build_local.sh        # Catkin build with vendor workspace setup
├── check_local.sh        # ROS package/dependency check
├── MANIFEST_SHA256.txt   # Checksums of this source release
└── README.md
```

**Runtime data** such as inspection profiles, result images, missions, calibration baselines and log files may be written **outside** the repository (`~/Documents/stip_inspection_data`, `~/Documents/stip_dashboard_data`, `~/.ros`). They are not generally part of Git. The two existing map pairs and the calibrated thermal alignment file **are included** in this initial snapshot; review these site-specific files before publishing publicly.

## Troubleshooting and verification

```bash
# Offline static code/XML/asset verification (does not need a running ROS master)
python3 tools/verify_workspace.py

# Check ROS package visibility and dependencies
./check_local.sh

# Check manager status when ROS is active
bash src/stip_rosmaster_manager/tools/stip_stage.sh status

# Check RGB, thermal and map-frame pose topics
rostopic hz /inspection_camera/image_raw/compressed
rostopic hz /thermal/image_color
rosrun tf tf_echo map base_link
```

Missing TF, stale localization, arm ownership, serial/I²C access or absent ROS packages must be resolved on the robot, not by relaxing motion safety rules. See [`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md).

## Documentation

- [`docs/INSTALLATION.md`](docs/INSTALLATION.md) — environment, build, hardware and first boot
- [`docs/OPERATOR_GUIDE.md`](docs/OPERATOR_GUIDE.md) — daily operation, inspection and calibration
- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) — subsystem ownership, ROS interfaces, external data
- [`docs/LAUNCH_CATALOG.md`](docs/LAUNCH_CATALOG.md) — launch inventory and when to use it
- [`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md) — diagnostic commands and known constraints
- [`docs/GITHUB_RELEASE.md`](docs/GITHUB_RELEASE.md) — safe first push and future updates
- [`docs/RELEASE_AUDIT.md`](docs/RELEASE_AUDIT.md) — exactly what the cleanup retained or excluded
- [`INTEGRATION_MQ6_MLX90640.md`](INTEGRATION_MQ6_MLX90640.md) — low-level sensor details

## Licensing and release status

No open-source license was selected for this first source snapshot. **Do not assume permission to redistribute third-party robot meshes, the TensorRT model engine or vendor software.** Create a private repository initially; review provenance and choose a license for code you own before publication. This source is for research/education and supervised robotics; it is not certified for fire detection, gas protection, or unattended operation.
