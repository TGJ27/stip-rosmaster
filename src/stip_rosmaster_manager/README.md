# stip_rosmaster_manager — STIP stage manager and dashboard

This package owns the system-wide startup/retry policy, original Jetson GUI, and standalone HTTP dashboard. It **does not replace** the base, camera, QR, fire, thermal or gas drivers in their own ROS packages.

## Web/headless mode (recommended)

```bash
cd ~/Documents/stip_rosmaster_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash --extend
./src/stip_rosmaster_manager/scripts/start_dashboard.sh
```

The launcher listens on `0.0.0.0:8765` unless overridden, can attach to an existing manager, and otherwise launches `dashboard_system.launch` through the headless `stip_dashboard_stage_manager.py` adapter. Set `STIP_AUTO_START_SYSTEM=0` to disable that startup, or `STIP_DASHBOARD_BIND=127.0.0.1` to keep the HTTP endpoint local. An optional `STIP_DASHBOARD_TOKEN` protects HTTP actions; it is **not enabled by default**.

## Original GUI ROS manager (alternative)

```bash
roslaunch stip_rosmaster_manager launcher.launch
# OR the staged full-system manager:
roslaunch stip_rosmaster_manager system.launch
```

Do **not** run `system.launch` and `dashboard_system.launch` at the same time. Both use `/stip_stage_manager` and can start duplicate owners of robot drivers/camera.

### Stage operations

```bash
bash src/stip_rosmaster_manager/tools/stip_stage.sh status
bash src/stip_rosmaster_manager/tools/stip_stage.sh restart camera
bash src/stip_rosmaster_manager/tools/stip_stage.sh restart qr
bash src/stip_rosmaster_manager/tools/stip_stage.sh restart fire
bash src/stip_rosmaster_manager/tools/stip_stage.sh restart gas
bash src/stip_rosmaster_manager/tools/stip_stage.sh restart thermal
bash src/stip_rosmaster_manager/tools/stip_stage.sh shutdown
```

The manager supervises independent stages, with configurable retry budgets and manual-only recovery after specific failures. Base bringup faults require operator intervention. Camera/QR/fire/thermal/gas stages use their own ROS packages; never start duplicates from outside while managed.

## Calibration tools

`calibrate_linear.launch` and `calibrate_angular.launch` use the STIP base and start with `start_test=false`; use `rqt_reconfigure` deliberately after securing the robot. `calibrate_imu.launch` uses `imu_calib`. Calibration results do **not** automatically adjust every downstream odometry parameter.

See the root [`README.md`](../../README.md), [`docs/OPERATOR_GUIDE.md`](../../docs/OPERATOR_GUIDE.md), and [`docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md).
