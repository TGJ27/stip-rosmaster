Jetson Orin fire/smoke detector TensorRT model
==============================================

Current v1.0.0 source release includes:
  fire_smoke_yolov8s.engine

Expected: YOLOv8s detection, class 0 = fire, class 1 = smoke,
1x3x416x736 (H x W) FP16 TensorRT engine. This serialized engine was
built for a particular Jetson/TensorRT environment; do not assume it
will load on another JetPack, TensorRT, CUDA or GPU combination.

The engine and its training/model provenance may carry licensing terms
separate from the source code. Verify redistribution rights before
making the GitHub repository public.
