cmake_minimum_required(VERSION 2.8.3)
project(rv6l_3d_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
 FILES
 RV6L_positions.msg
 RV6L_position.msg
)

generate_messages(
 DEPENDENCIES
 std_msgs
)
catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)
