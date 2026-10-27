#!/bin/bash
set -e
MAP_NAME="${1:-my_map}"
MAP_DIR="$(rospack find stip_rosmaster_nav)/maps"
mkdir -p "$MAP_DIR"
echo "Saving map to: $MAP_DIR/$MAP_NAME"
rosrun map_server map_saver -f "$MAP_DIR/$MAP_NAME"
