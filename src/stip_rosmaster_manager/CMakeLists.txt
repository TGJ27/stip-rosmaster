cmake_minimum_required(VERSION 3.0.2)
project(stip_rosmaster_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  dynamic_reconfigure
  tf
  nav_msgs
)

generate_dynamic_reconfigure_options(
  cfg/CalibrateLinear.cfg
  cfg/CalibrateAngular.cfg
)

catkin_package(CATKIN_DEPENDS rospy std_msgs std_srvs sensor_msgs geometry_msgs dynamic_reconfigure tf nav_msgs)

catkin_install_python(PROGRAMS
  scripts/stip_stage_manager.py
  scripts/stip_launcher_gui.py
  scripts/stip_stage_watcher.py
  scripts/calibrate_linear.py
  scripts/calibrate_angular.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS tools/stip_stage.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
