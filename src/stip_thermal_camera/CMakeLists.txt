cmake_minimum_required(VERSION 3.0.2)
project(stip_thermal_camera)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs cv_bridge)
catkin_package(CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge)
catkin_install_python(PROGRAMS
  scripts/thermal_node.py scripts/thermal_viewer.py
    scripts/thermal_manual_alignment.py
    scripts/thermal_rgb_overlay.py
    scripts/thermal_temp_calibrate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
