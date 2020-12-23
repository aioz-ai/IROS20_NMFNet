cmake_minimum_required(VERSION 2.8.3)
project(beetlebot_description)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)


find_package(catkin REQUIRED)

catkin_package(
    DEPENDS
    roscpp
    gazebo_ros
)
