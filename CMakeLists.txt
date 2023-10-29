cmake_minimum_required(VERSION 3.0.2)
project(multi_copter_ctrl)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  multi_copter_msgs
  rospy
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)