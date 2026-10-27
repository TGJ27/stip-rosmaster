# stip_rosmaster_nav

## Individual tests

LiDAR only:

```bash
roslaunch stip_rosmaster_nav lidar.launch
```

Mapping only (base + LiDAR must already be running):

```bash
roslaunch stip_rosmaster_nav mapping.launch
```

Navigation only (base + LiDAR must already be running):

```bash
roslaunch stip_rosmaster_nav navigation.launch map:=my_map
```

## Full real-robot launches

```bash
roslaunch stip_rosmaster_nav mapping_full.launch
roslaunch stip_rosmaster_nav navigation_full.launch map:=my_map
```
