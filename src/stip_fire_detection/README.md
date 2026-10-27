# stip_fire_detection — native fire/smoke detector

Jetson Orin C++/CUDA/TensorRT fire and smoke detection using the existing `models/fire_smoke_yolov8s.engine` (FP16, 416×736, class 0 = fire, class 1 = smoke).

This source snapshot includes only the **native C++ detector** entrypoints. Legacy Python detector launch instructions from an older release are not applicable here.

## Entrypoints

- `roslaunch stip_fire_detection stage_fire.launch` — manager-owned detector, reusing the shared camera.
- `roslaunch stip_fire_detection fire_detection_cpp.launch` — independent detector/camera test (may open shared hardware). If another node already owns the camera, use `start_camera:=false` as supported by its launch arguments.

## Relevant topics

`/fire_detection/detections`, `/fire_detection/status`, `/fire_detection/active`, `/fire_detection/fire_active`, `/fire_detection/smoke_active`, `/fire_detection/inference_ms` and `/fire_detection/pipeline_ms`.

Config: `config/fire_cpp.yaml`. Confidence and alarm-confirmation thresholds are **distinct**. Changes require the owning detector to restart before the new parameters are active. See the dashboard Settings panel for managed changes.

## Platform requirements

The `CMakeLists.txt` requires CUDA 11.4, TensorRT headers/libraries, OpenCV, and GStreamer development packages and compiles for CUDA `sm_87` (Jetson Orin). This is not a generic desktop build. The bundled serialized engine is platform/version-specific; validate it on your target before operation.

```bash
cd ~/Documents/stip_rosmaster_ws
./build_local.sh
source devel/setup.bash --extend
rostopic hz /fire_detection/detections
rostopic echo /fire_detection/inference_ms
```

The fire detector identifies AI candidates. **Confirmed fire** is decided separately by `stip_fire_inspection` using fresh detections and calibrated MLX90640 target temperature. Do not use this experimental stack as a certified fire-alarm or safety system.
