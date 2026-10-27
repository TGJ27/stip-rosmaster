# Changelog

## 1.0.0 — 2026-10-27

First GitHub-ready source snapshot from the operator's uploaded Jetson ROS workspace.

- Retained the 12 ROS packages, 36 ROS launch files, working modular dashboard, current XML/YAML, firmware, meshes, TensorRT engine, thermal/RGB calibration and two saved map pairs.
- Excluded eight local dashboard backup directories, two old config backups and generated Python bytecode caches; no active driver or ROS node implementation removed.
- Converted `my_map.yaml` image reference from a Jetson-specific absolute file path to the local relative `my_map.pgm` path.
- Added documentation, `.gitignore`, `.gitattributes`, version metadata and a regenerated source checksum manifest.
- Preserved executable permissions and the canonical catkin `src/CMakeLists.txt` symlink in the release archive.

No live hardware verification was conducted as part of the GitHub-export cleanup. See `docs/RELEASE_AUDIT.md`.
