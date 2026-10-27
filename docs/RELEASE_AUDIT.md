# v1.0.0 cleanup and verification report

**Source:** operator-uploaded `here(1).zip` (27 October 2026). **Purpose:** create an initial GitHub-ready source release without rewriting functioning ROS robot code.

## Changes made

- Excluded **59 non-source files** in total: old patch/update backup snapshots, generated Python bytecode and two superseded YAML backup copies. Excluded directories themselves are not needed for a fresh checkout.
- Kept **all 12 ROS packages**, all existing launch files, Arduino POP32i firmware, the TensorRT engine, ROSMASTER meshes, robot maps and current calibrated thermal alignment.
- Restored `src/CMakeLists.txt` as the actual Unix **symlink** to the system catkin top-level CMake file, not a flattened text file. `build_local.sh` retains a repair fallback for third-party archives.
- Rewrote the stale root README to match the currently uploaded web-dashboard workspace rather than an October 21 cleanup installer absent from this archive.
- Replaced the stale/inconsistent fire detector README and its engine note with the actual native C++ launch options present in this snapshot; refreshed manager documentation to include headless web mode.
- Updated `src/stip_rosmaster_nav/maps/my_map.yaml` to reference `image: my_map.pgm` instead of `/home/jetson/.../my_map.pgm`, preserving the same file while enabling relocatable clones.
- Added `.gitignore`, `.gitattributes`, `VERSION`, `CHANGELOG.md`, installation/operation/architecture/launch/troubleshooting/GitHub docs.
- Regenerated `MANIFEST_SHA256.txt` for the exact cleaned source tree (excluding the manifest itself and the system-targeted symlink).

## Intentionally retained

- `src/stip_rosmaster_nav/maps/my_map.yaml` + `.pgm` and `test.yaml` + `.pgm` (the operator's current map data; the latter is not proven disposable).
- `src/stip_thermal_camera/config/thermal_rgb_alignment.yaml` (site-specific calibration used by fire fusion and overlays).
- `src/stip_fire_detection/models/fire_smoke_yolov8s.engine` (~24.5 MB; required by the native fire launch, tied to Jetson/TensorRT version).
- `src/stip_rosmaster_description/meshes/` (~99 MB) for robot URDF visualization.
- Existing independent calibration and testing scripts/launch files; older UI tools may be useful for robot diagnosis and aren't safely deletable merely because the web dashboard supersedes them.
- Live ROS message/service contracts, driver logic, camera publishers, inspector, fusion and navigation implementation.

## Important issues in the uploaded snapshot

1. The original `README.md` referred to `docs/LAUNCH_CATALOG.md`, `docs/CLEANUP_REPORT.md`, `install_clean.py` and a different source state; those files were not in the upload. The release supplies current documentation instead.
2. The original checksum manifest failed on missing documentation and several modified configuration/source files. A correct manifest is generated for this release.
3. `src/CMakeLists.txt` is represented as a Git-friendly symlink; if the target `/opt/ros/noetic/...` does not exist on a non-ROS machine, offline static checking handles this as expected.
4. The original fire package README described a legacy Python detector and launch files that are not present in the current archive. Only current native launch files are documented now.
5. The checked-in engine, robot meshes, map and calibration data may be restricted/proprietary/site-sensitive; initial visibility should be private.
6. Robot behavior cannot be validated on a non-Jetson system, and static tests do not prove sensor, servo, serial or safety performance. A live smoke-test checklist is in `INSTALLATION.md` and `OPERATOR_GUIDE.md`.

**No active ROS Python/C++ algorithms, node entrypoints or launch configurations were deleted.** The cleanup is deliberately conservative.
