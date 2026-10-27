# Operator guide — STIP ROSMASTER 1.0.0

## Pre-flight

1. Inspect battery/wiring, clear the operating zone and make an accessible **physical** emergency-stop plan.
2. Verify the Jetson and ROS stage manager, robot bringup, LiDAR, camera and relevant sensors show current data.
3. Confirm the browser is connected to the trusted local Jetson Wi-Fi/hotspot and other clients are not controlling the base simultaneously.
4. Test base motion with wheels raised before letting the robot move on the ground.
5. Treat Fire and MQ-6 as **experimental monitoring**; not life-safety instruments.

## Home and stage manager

Home shows service health, current readiness and fault information. The web server can remain available even if ROS is offline; **HTTP 200 does not mean robot motion is safe**. The headless stage manager controls startup/retry. Do not manually start the same robot driver/camera or QR/fire stage in another terminal unless the owning stage is stopped.

## Maps

- Create a unique map name; **Start Mapping** is a state-changing action, not a button that needs repeated clicking.
- Arm teleop, drive slowly and scan from multiple directions. Release keys to command zero velocity.
- Save the map and verify the matching `.yaml`/`.pgm` pair in `src/stip_rosmaster_nav/maps/`.
- Deleting a map is destructive; verify no saved mission still uses it.
- Switching pages while mapping must be deliberate. If prompted, stop/save/abort and confirm before leaving.

## Mission editor and localization

- Select a saved map and enter the initial-pose screen.
- The directional triangle represents heading. If its displayed position/orientation matches the physical robot, Continue is an operator acknowledgement. Otherwise, drag to the desired map pose/heading and publish **Localize** before continuing.
- This does not remove the requirement for valid `map → odom → base_link` TF and actual navigational localization when the robot moves.
- In Mission Editor, use teleop to move the robot to a location, stop fully, then **Add Current Pose** to save that map-frame X/Y/yaw. The waypoint triangle shows heading.
- Add optional QR and pause tasks to the waypoint, then **Save Mission**.

## QR enrollment, test and gallery

1. From the selected waypoint add a QR task, open the large live-camera view, aim with W/A/S/D.
2. Click **Enroll QR**. The QR detector must find a readable code before an ROI can be saved.
3. Inspect the green QR outline, draw the separate indicator ROI over the desired part of the image, then **Save Enrollment**.
4. Profile data is saved under `~/Documents/stip_inspection_data/profiles/`, separate from the waypoint/mission JSON. Re-enrollment of a profile name can update the stored reference on Save; editing a waypoint does not instantly change the stored profile.
5. Use **View Profile** to verify saved QR value, annotated image and servo angles; **Test Inspection** calls `/inspection/run` with the saved profile and does not navigate the robot base.
6. Open the separate **Inspection Gallery** to view newer-to-older inspection runs with annotated images and indicator crops. Gallery closing does not cancel ROS services already executing.

The existing `stip_qr_inspection` service owns QR scanning/tracking and inspection results. The dashboard does not require a second QR detector to run alongside it during patrol.

## Patrol

- Load a saved mission, confirm the initial pose, and only then Start Patrol.
- **Start/Pause/Resume/Stop** control a ROS-supervised mission. A completed mission does not imply the robot is safe to command again without checking mode/interlocks.
- Live Inspection shows RGB, thermal and fire/gas state. Use Expand Live Camera for a larger calibrated display. The calibrated RGB/thermal view comes from `/thermal/rgb_overlay`; it is **not** a browser-scaled thermal thumbnail.
- Activity & Evidence has its own scrollable event list, inspection gallery and fire evidence; it does not scroll the video offscreen.
- Leaving Patrol should prompt **Stop Patrol & Leave**, stop dashboard-owned motion/navigation, then request Fire Monitoring and auto-tracking OFF. If shutdown is not acknowledged, the page should report it instead of silently closing.

## Fire and gas monitoring

- Fire Monitoring enables the existing AI+thermal fire-controller workflow; smoke alone is not a thermally confirmed fire. A valid heat target must correspond to AI-detected fire geometry.
- Fire auto-aim can require exclusive ownership of `/TargetAngle` and an acknowledged stationary/paused robot. An enabled checkbox is **not** proof that the arm is actively tracking.
- Confirmed fire may latch a safety stop even after monitoring is disabled. Clear it using the verified ROS acknowledgement flow, never by bypassing motion restrictions.
- Fire evidence is written when the controller reports confirmation, not for every ordinary RGB frame.
- MQ-6 calibration must be performed after warm-up in **known clean air**. Its ADC delta is relative, not an absolute LPG concentration or gas safety reading.

## Settings

- Fire and smoke detector confidence changes target the C++ detector configuration; restart the managed detector for changes to become active.
- Fire tracking speed settings target the **existing** `stip_fire_inspection/config/inspection.yaml` movement-step/command-interval parameters and require controller restart. Stop Patrol/monitoring first.
- Calibration includes site-specific thermal/RGB alignment (`src/stip_thermal_camera/config/thermal_rgb_alignment.yaml`), servo limits and robot odometry. Back up real-world calibration data before editing; revalidate after camera mounting or sensor changes.

## Important stop conditions

Stop immediately for unexpected motion, missing localization, stale camera/thermal data, navigation/servo control conflicts, unexplained fire hold, overcurrent or actuator overheating. A browser-side software stop is not a hardware isolation mechanism.
