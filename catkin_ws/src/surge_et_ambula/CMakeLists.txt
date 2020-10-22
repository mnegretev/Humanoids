cmake_minimum_required(VERSION 2.8.3)
project(surge_et_ambula)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# For earthquake_plugin plugin
add_library(plannar_mover_plugin src/plannar_mover.cpp)
target_link_libraries(plannar_mover_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES surge_et_ambula
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

