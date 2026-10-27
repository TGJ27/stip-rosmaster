cmake_minimum_required(VERSION 3.0.2)
project(stip_camera_arm)

find_package(catkin REQUIRED COMPONENTS
  rospy
  stip_rosmaster_msgs
)

catkin_package(CATKIN_DEPENDS rospy stip_rosmaster_msgs)

catkin_install_python(PROGRAMS
  scripts/camera_arm_teleop.py
  scripts/camera_arm_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
