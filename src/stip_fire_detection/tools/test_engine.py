#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse

# Import OpenCV before Ultralytics on this Jetson.
import cv2
from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--engine", required=True)
    parser.add_argument("--image", required=True)
    parser.add_argument("--height", type=int, default=416)
    parser.add_argument("--width", type=int, default=736)
    parser.add_argument("--conf", type=float, default=0.10)
    args = parser.parse_args()

    model = YOLO(args.engine, task="detect")

    results = model.predict(
        source=args.image,
        imgsz=(args.height, args.width),
        conf=args.conf,
        verbose=True
    )

    result = results[0]
    print("names:", result.names)
    print("box count:", 0 if result.boxes is None else len(result.boxes))

    if result.boxes is not None:
        for box in result.boxes:
            cls_id = int(box.cls[0])
            print(
                "class=", result.names[cls_id],
                "conf=", round(float(box.conf[0]), 4),
                "xyxy=", [round(float(v), 1) for v in box.xyxy[0]]
            )

    print("ENGINE TEST: PASS")


if __name__ == "__main__":
    main()
