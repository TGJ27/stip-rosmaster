cmake_minimum_required(VERSION 3.0.2)
project(stip_gas_sensor)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs std_srvs)
catkin_package(CATKIN_DEPENDS rospy std_msgs std_srvs)
catkin_install_python(PROGRAMS scripts/gas_bridge.py   scripts/mq6_baseline_gui.py
    scripts/mq6_baseline_calibrate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
