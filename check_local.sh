#!/bin/bash
set -e

ROOT="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/noetic/setup.bash

VENDOR_SETUP="$HOME/software/library_ws/devel/setup.bash"
if [ -f "$VENDOR_SETUP" ]; then
  source "$VENDOR_SETUP" --extend
fi

if [ -f "$ROOT/devel/setup.bash" ]; then
  source "$ROOT/devel/setup.bash" --extend
fi

packages=(
  stip_rosmaster_msgs
  stip_rosmaster_description
  stip_rosmaster_bringup
  stip_rosmaster_nav
  stip_inspection_camera
  stip_camera_arm
  stip_qr_inspection
  stip_fire_detection
  stip_fire_inspection
  stip_gas_sensor
  stip_thermal_camera
  stip_rosmaster_manager
)

for p in "${packages[@]}"; do
  if rospack find "$p" >/dev/null 2>&1; then
    echo "OK      $p -> $(rospack find "$p")"
  else
    echo "MISSING $p"
  fi
done

if rospack find ydlidar_ros_driver >/dev/null 2>&1; then
  echo "OK      ydlidar_ros_driver -> $(rospack find ydlidar_ros_driver)"
else
  echo "WARNING ydlidar_ros_driver is not visible"
fi

ENGINE="$ROOT/src/stip_fire_detection/models/fire_smoke_yolov8s.engine"
if [ -f "$ENGINE" ]; then
  echo "OK      fire TensorRT engine -> $ENGINE"
else
  echo "WARNING fire TensorRT engine missing -> $ENGINE"
fi

if python3 -c 'import Rosmaster_Lib' >/dev/null 2>&1; then
  echo "OK      Rosmaster_Lib import"
else
  echo "WARNING Rosmaster_Lib is not importable in this shell"
fi

# Optional calibration UIs (available only when these ROS packages are installed).
for p in rqt_reconfigure imu_calib; do
  if rospack find "$p" >/dev/null 2>&1; then
    echo "OK      $p -> $(rospack find "$p")"
  else
    echo "WARNING $p is not visible (required only for its calibration tool)"
  fi
done
