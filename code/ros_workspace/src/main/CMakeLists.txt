cmake_minimum_required(VERSION 3.0.2)
project(rv6l_main)

find_package(catkin REQUIRED COMPONENTS
  realsense2_camera
  darknet_ros_3d
  rv6l_3d_msgs
  rv6l_pick_and_place
  forward_command_controller
  joint_state_controller
  joint_trajectory_controller
  position_controllers
  rqt_joint_trajectory_controller
  rsv_cartesian_interface
  rsv_joint_interface
  rv6l
  rv6l_cell
  rv6l_cell_config
  rv6l_config
  opencv_apps
  pcl_ros
  nlopt
)
catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

