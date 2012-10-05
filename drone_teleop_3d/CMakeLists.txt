cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_add_executable( bin/drone_teleop_3d
  src/spacenav_node.cpp
  )
