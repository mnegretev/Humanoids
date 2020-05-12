cmake_minimum_required(VERSION 2.8.3)
project(humanoid)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  humanoid_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES humanoid
  CATKIN_DEPENDS roscpp rospy std_msgs humanoid_msgs 
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(humanoid src/Humanoid.cpp)

add_dependencies(humanoid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(humanoid ${catkin_LIBRARIES})
 
