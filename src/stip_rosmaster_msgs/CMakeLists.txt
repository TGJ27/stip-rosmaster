cmake_minimum_required(VERSION 3.0.2)
project(stip_rosmaster_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(FILES ArmJoint.msg)
add_service_files(FILES RobotArmArray.srv)
generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)
