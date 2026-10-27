# Troubleshooting

Run hardware checks on the Jetson, in a shell that has sourced the correct ROS environment. Do **not** bypass an active fire/motion interlock to make a UI button work.

| Symptom | Check |
| --- | --- |
| Dashboard reachable, ROS offline | `rosnode list`, `bash src/stip_rosmaster_manager/tools/stip_stage.sh status`, headless manager log `~/.ros/stip_dashboard/stage_manager.log` |
| `0.0.0.0:8765` but remote browser cannot connect | `hostname -I`, `ss -ltnp | grep 8765`, same Wi-Fi/hotspot, firewall, Jetson IP (not `0.0.0.0`) |
| Map missing | `ls src/stip_rosmaster_nav/maps`; YAML+PGM pair must match; check YAML `image:` location |
| AMCL pose marker wrong | Check `rostopic echo -n 1 /amcl_pose`, `rosrun tf tf_echo map base_link`, LiDAR TF, odometry and orientation. Visual confirmation isn't sensor validation |
| Teleop rejected | Stop/exit the other motion owner; arm teleop explicitly in dashboard; release keys; inspect fire hold and Patrol state |
| QR not detected | `rostopic hz /inspection_camera/image_raw/compressed`, image sharpness/QR scale/illumination, `pyzbar`+ZBar dependency; verify only one camera driver |
| Camera-arm control or tracking blocked | `rostopic info /TargetAngle` for competing publishers; check `/ArmAngleUpdate`, servo limits, stationary pause acknowledgement |
| Fire reports `UNVERIFIED_TIMEOUT` | Check fresh fire AI detections, thermal samples, overlap/registration and target temperature; unverified is **not** confirmed fire |
| Calibrated RGB/thermal overlay unavailable | `rostopic hz /thermal/rgb_overlay`, correct saved `thermal_rgb_alignment.yaml`, I²C sensor and RGB camera streaming |
| Gas bridge silent | `ls /dev/ttyACM* /dev/ttyUSB*`, permissions, POP32i firmware, stable 115200 USB serial |
| Settings says fire monitoring ON after exit | Confirm stop workflow finished; inspect actual ROS controller status before reconfiguring. Confirmed fire holds remain latched |
| TensorRT/compile errors | Check JetPack/CUDA/TRT versions, `sm_87` target, installed engine compatibility, GStreamer/OpenCV headers and vendor workspace |
| Stale dashboard after updating | Stop old process, restart `start_dashboard.sh`, browser hard-refresh, inspect JavaScript console errors |

## Useful ROS commands

```bash
source /opt/ros/noetic/setup.bash
source ~/Documents/stip_rosmaster_ws/devel/setup.bash --extend
rosnode list
rosservice list | grep -E '(inspection|fire|stip|gas)'
rostopic hz /inspection_camera/image_raw/compressed
rostopic hz /thermal/image_color
rostopic hz /thermal/rgb_overlay
rostopic info /TargetAngle
rostopic echo -n 1 /fire_inspection/status_json
rostopic echo -n 1 /gas_sensor/raw
```

Stage operations should go through the owner:

```bash
bash src/stip_rosmaster_manager/tools/stip_stage.sh status
bash src/stip_rosmaster_manager/tools/stip_stage.sh restart camera
bash src/stip_rosmaster_manager/tools/stip_stage.sh restart qr
bash src/stip_rosmaster_manager/tools/stip_stage.sh restart fire
```

`restart` requests depend on the active stage-manager mode. Do not start an independent second manager to make a service appear. Consult `INTEGRATION_MQ6_MLX90640.md` for hardware-specific wiring/thermal bus diagnostics.
