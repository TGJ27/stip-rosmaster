# Architecture and component ownership

## Boundaries

The dashboard is a **web interface and ROS adapter**, not a replacement hardware driver. Source files are grouped by ownership so stages can be independently started, diagnosed and recovered. Never launch multiple owners of the same physical device.

| ROS package | Responsibility / primary interface |
| --- | --- |
| `stip_rosmaster_manager` | GUI/headless stage manager, stage health/retry, dashboard HTTP + ROS worker, calibration tools |
| `stip_rosmaster_bringup` | Yahboom base hardware, odometry, IMU/EKF, `/cmd_vel`, servo driver |
| `stip_rosmaster_nav` | LiDAR integration, mapping, AMCL navigation, map save/load, RViz |
| `stip_rosmaster_description` | URDF, meshes and robot frame geometry |
| `stip_rosmaster_msgs` | Shared `ArmJoint` message and robot-arm service |
| `stip_inspection_camera` | Shared MJPEG camera publisher (`/inspection_camera/image_raw/compressed`) |
| `stip_camera_arm` | Standalone manual pan/tilt clients via `/TargetAngle` |
| `stip_qr_inspection` | QR enrollment, pan/tilt scan/tracking, indicator ROI, `/inspection/run`, `/inspection/geometry` |
| `stip_fire_detection` | Native C++/CUDA/TensorRT fire and smoke detections (`/fire_detection/detections`) |
| `stip_fire_inspection` | AI/thermal fusion, auto-aim, confirmed/unverified state, alarm and acknowledgement |
| `stip_thermal_camera` | MLX90640 `/thermal/image_raw`, color rendering and calibrated `/thermal/rgb_overlay` |
| `stip_gas_sensor` | POP32i/MQ-6 serial bridge (`/gas_sensor/raw`, `/gas_sensor/change_percent`, calibrate service) |

## Launch ownership

- **Web mode:** `start_dashboard.sh` supervises the HTTP process and can start the **headless** manager `dashboard_system.launch`. The manager runs the existing startup/retry protocol.
- **Original full ROS mode:** `system.launch` manages the original UI stage. Do **not** start it beside the headless dashboard manager.
- **Independent testing:** each subsystem's `*.launch` starts only its documented requirements. Standalone `mapping_full.launch`, `navigation_full.launch`, `inspection.launch` and `fire_inspection_live.launch` may start overlapping camera/base/serial processes; run one ownership tree at a time.
- **Arm ownership:** camera-arm GUI, QR and fire auto-tracking all use `/TargetAngle`. The fire controller requires exclusive access to avoid competing servo commands. Keep idle publishers released and respect its safe-pause acknowledgement.

## Data flow (simplified)

```text
Yahboom robot / LiDAR  -> ROS bringup, TF, SLAM, AMCL, move_base
USB MJPEG camera       -> stip_inspection_camera -> /inspection_camera/image_raw/compressed
RGB stream             -> QR inspector / C++ fire AI / dashboard MJPEG bridge
MLX90640               -> stip_thermal_camera -> /thermal/image_raw
RGB + thermal + saved H -> ROS calibrated /thermal/rgb_overlay
AI fire + thermal      -> stip_fire_inspection fusion -> status, stop/hold, evidence
POP32i UART MQ-6       -> stip_gas_sensor -> raw ADC and baseline delta
Manager & ROS worker   -> dashboard server (:8765) -> browser on local network
```

## Source versus mutable runtime data

- **In repository:** source, launch/config, robot meshes, compiled TensorRT engine snapshot, firmware, `my_map`/`test` map pairs and saved RGB–thermal alignment YAML.
- **Outside repository:** `~/Documents/stip_inspection_data/` holds QR profiles and inspection/fire evidence; `~/Documents/stip_dashboard_data/` holds mission definitions; `~/.ros/` may contain logs, maps and MQ-6 calibration. Keep these in a separate private backup plan.
- **Hardware-specific:** `/dev/ttyACM0`, `/dev/i2c-1`, camera device, model engine and thermal homography must be checked per installation. Avoid publishing site maps/calibration or facility camera images publicly without permission.

## Safety invariant

Only the active ROS motion/inspection controller can authorize movement and arm ownership. A view transition or frontend checkbox does not prove backend completion. Stop/exit events must cancel owned goals, publish safe velocity, and wait for acknowledgement where supported; confirmed fire holds must not be silently cleared.
