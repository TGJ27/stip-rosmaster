# stip_inspection_camera

Publishes the original webcam MJPEG bytes without continuously decoding them.

## Camera only (reusable by other packages)

```bash
roslaunch stip_inspection_camera camera.launch
```

Topic:

```text
/inspection_camera/image_raw/compressed
```

## Camera test with local view

```bash
roslaunch stip_inspection_camera camera_test.launch
```

The test view decodes at half resolution by default, so a 1920x1080 camera is
shown as a manageable 960x540 preview.
