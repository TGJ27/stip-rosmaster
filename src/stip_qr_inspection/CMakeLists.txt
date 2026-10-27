cmake_minimum_required(VERSION 3.0.2)
project(stip_qr_inspection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  stip_rosmaster_msgs
  message_generation
)

add_service_files(FILES
  EnrollInspection.srv
  RunInspection.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs sensor_msgs std_msgs stip_rosmaster_msgs message_runtime
)

catkin_install_python(PROGRAMS
  scripts/qr_inspection_node.py
  scripts/inspection_ui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Source-backed GStreamer helper for devel and install-space execution.
install(FILES scripts/stip_hw_jpeg.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
