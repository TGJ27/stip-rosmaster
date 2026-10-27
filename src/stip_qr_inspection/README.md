# stip_qr_inspection — QR inspection subsystem

This package owns QR/indicator recognition, tracking and the local inspection
UI. It does **not** launch fire/smoke detection or navigation.

### Standalone

```bash
roslaunch stip_qr_inspection inspection.launch
```

That starts this subsystem with the **required dependencies**:
- shared MJPEG inspection camera (for images)
- ROSMASTER bringup (for pan/tilt camera-arm servo commands)

If those processes already run, avoid duplicates:

```bash
roslaunch stip_qr_inspection inspection.launch start_camera:=false start_robot:=false
```

`stage_ui.launch` and `stage_qr.launch` launch the two QR subsystem processes
*without* dependencies; the system manager uses them for independent startup
and recovery. They are not normal standalone entrypoints.

The UI can optionally display fire detections from ROS if another package
publishes them, but **QR never starts fire detection**.

- **I**: request inspection; QR decode is on demand.
- **E**: enrollment; QR decode starts temporarily.
- **W/A/S/D/H**: camera-arm control (bringup required).
- **Q**: close standalone UI; in manager mode, shut down the whole application.
- Hardware JPEG decoding via NVIDIA GStreamer by default; OpenCV fallback.
