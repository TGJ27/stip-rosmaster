#!/bin/bash
set -e

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

source /opt/ros/noetic/setup.bash

CATKIN_TOPLEVEL="/opt/ros/noetic/share/catkin/cmake/toplevel.cmake"
SRC_CMAKE="$ROOT/src/CMakeLists.txt"

# Some ZIP/Windows copy tools flatten the catkin CMakeLists symlink into
# a one-line text file containing only its target path. Repair that case.
if [ ! -e "$SRC_CMAKE" ]; then
  echo "Creating catkin workspace CMakeLists symlink..."
  ln -s "$CATKIN_TOPLEVEL" "$SRC_CMAKE"
elif [ ! -L "$SRC_CMAKE" ] && [ "$(tr -d '\r\n' < "$SRC_CMAKE")" = "$CATKIN_TOPLEVEL" ]; then
  echo "Repairing flattened catkin workspace CMakeLists symlink..."
  rm -f "$SRC_CMAKE"
  ln -s "$CATKIN_TOPLEVEL" "$SRC_CMAKE"
fi

VENDOR_SETUP="$HOME/software/library_ws/devel/setup.bash"
if [ -f "$VENDOR_SETUP" ]; then
  source "$VENDOR_SETUP" --extend
  echo "Using Yahboom/vendor workspace: $VENDOR_SETUP"
else
  echo "WARNING: vendor workspace not found: $VENDOR_SETUP"
  echo "         Rosmaster_Lib / ydlidar_ros_driver must still be available elsewhere."
fi

catkin_make "$@"

echo
echo "Build complete. Run:"
echo "  source $ROOT/devel/setup.bash --extend"
