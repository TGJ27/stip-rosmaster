cmake_minimum_required(VERSION 3.0.2)
project(stip_inspection_camera)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS rospy sensor_msgs)

catkin_install_python(PROGRAMS
  scripts/inspection_camera_node.py
  scripts/camera_view.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
