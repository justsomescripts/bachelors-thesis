cmake_minimum_required(VERSION 3.0.2)
project(rv6l_pick_and_place)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  interactive_markers
  moveit_core
  moveit_ros_perception
  moveit_ros_planning_interface
  moveit_visual_tools
  pluginlib
  roscpp
  std_msgs
  nlopt
  #moveit-chomp-optimizer-adapter
)


catkin_package(
)
include_directories(

  ${catkin_INCLUDE_DIRS}
)
