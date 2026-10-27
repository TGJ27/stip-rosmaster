# ROS launch catalog (from the uploaded workspace)

**Do not launch overlapping owning workflows at the same time.** Each table entry is a real `.launch` file in the v1.0.0 source; the short purpose is orientation, and the launch file is authoritative for arguments and dependencies.

| Package | Entry point | When to use |
| --- | --- | --- |
| `stip_rosmaster_manager` | `dashboard_system.launch` | Headless staged ROS manager for the web dashboard |
| `stip_rosmaster_manager` | `system.launch` | Original manager with ROS UI stage; not alongside headless manager |
| `stip_rosmaster_manager` | `launcher.launch` | Desktop workflow GUI |
| `stip_rosmaster_manager` | `calibrate_linear.launch` | STIP-only straight-line odometry test, operator-triggered |
| `stip_rosmaster_manager` | `calibrate_angular.launch` | STIP-only angular odometry test, operator-triggered |
| `stip_rosmaster_manager` | `calibrate_imu.launch` | Stationary IMU calibration workflow |
| `stip_rosmaster_bringup` | `bringup.launch` | Base/IMU/EKF and robot hardware |
| `stip_rosmaster_nav` | `lidar.launch` | LiDAR driver only |
| `stip_rosmaster_nav` | `mapping.launch` | GMapping (expects base and LiDAR) |
| `stip_rosmaster_nav` | `mapping_full.launch` | Standalone full mapping with base/LiDAR |
| `stip_rosmaster_nav` | `navigation.launch` | Navigation using existing base/LiDAR |
| `stip_rosmaster_nav` | `navigation_full.launch` | Standalone full navigation incl. base/LiDAR |
| `stip_rosmaster_nav` | `launch/library/*.launch` | Internal map saver, AMCL, mapping and move_base launch components |
| `stip_rosmaster_nav` | `launch/view/*.launch` | Visualization helpers |
| `stip_rosmaster_description` | `view_model.launch` | URDF/RViz visualization |
| `stip_inspection_camera` | `camera.launch` | Shared RGB camera publisher |
| `stip_inspection_camera` | `camera_test.launch` | Camera publisher + local test viewer |
| `stip_camera_arm` | `teleop.launch` | Standalone keyboard pan/tilt |
| `stip_camera_arm` | `teleop_gui.launch` | Standalone camera arm GUI |
| `stip_qr_inspection` | `inspection.launch` | Standalone QR inspector and necessary dependencies |
| `stip_qr_inspection` | `stage_qr.launch` | QR backend stage owned by manager |
| `stip_qr_inspection` | `stage_ui.launch` | Original ROS inspector UI stage |
| `stip_fire_detection` | `stage_fire.launch` | Native C++ TensorRT detector stage |
| `stip_fire_detection` | `fire_detection_cpp.launch` | Standalone detector test workflow |
| `stip_fire_inspection` | `fire_inspection.launch` | Fusion/controller stage |
| `stip_fire_inspection` | `fire_inspection_live.launch` | Attended full fusion/fire tracking demo; may launch many devices |
| `stip_thermal_camera` | `thermal_camera.launch` | MLX90640 acquisition |
| `stip_thermal_camera` | `thermal_test.launch` | Thermal sensor standalone test |
| `stip_thermal_camera` | `thermal_rgb_calibrate.launch` | RGB/thermal registration calibration |
| `stip_thermal_camera` | `thermal_rgb_view.launch` | Calibrated RGB+thermal preview |
| `stip_gas_sensor` | `gas_sensor.launch` | POP32i/MQ-6 bridge |
| `stip_gas_sensor` | `mq6_calibrate.launch` | Relative clean-air ADC baseline calibration |

Use `find src -name '*.launch' | sort` to enumerate all 36 actual launch files. ROS `$(find ...)` references resolve correctly only after sourcing the built catkin workspace and its dependencies.
