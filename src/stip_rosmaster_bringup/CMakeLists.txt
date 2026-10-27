cmake_minimum_required(VERSION 3.0.2)
project(stip_rosmaster_bringup)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  tf
)

generate_dynamic_reconfigure_options(
  cfg/PIDparam.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs nav_msgs roscpp rospy tf
)

include_directories(
  include/stip_rosmaster_bringup
  ${catkin_INCLUDE_DIRS}
)

add_executable(base_node src/base_node.cpp src/base.cpp)
target_link_libraries(base_node ${catkin_LIBRARIES})
add_dependencies(base_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  scripts/Mcnamu_X3plus.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS base_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch param cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
