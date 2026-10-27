# stip_camera_arm

Simple keyboard teleoperation for the 2-servo inspection camera arm.

## Standalone hardware test

```bash
roslaunch stip_camera_arm teleop.launch
```

By default this also starts:

```text
stip_rosmaster_bringup/bringup.launch
```

because the physical bus-servo command `/TargetAngle` is handled by the
ROSMaster driver.

If robot bringup is already running:

```bash
roslaunch stip_camera_arm teleop.launch start_robot:=false
```

## Controls

```text
A = pan left
D = pan right
W = tilt up
S = tilt down
H = home
Q = quit
```

Defaults:

```text
pan servo ID   = 1
tilt servo ID  = 2
pan home       = 90 deg
tilt home      = 0 deg
step           = 1 deg
run time       = 200 ms
pan limits     = 0..180 deg
tilt limits    = 0..90 deg
```

The node publishes `stip_rosmaster_msgs/ArmJoint` on:

```text
/TargetAngle
```
