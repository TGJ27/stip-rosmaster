cmake_minimum_required(VERSION 3.0.2)
project(stip_fire_inspection)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs geometry_msgs std_srvs cv_bridge stip_rosmaster_msgs)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/fire_inspection_node.py
  scripts/fire_inspection_viewer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
