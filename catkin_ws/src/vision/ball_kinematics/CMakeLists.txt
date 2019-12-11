cmake_minimum_required(VERSION 2.8.3)
project(ball_kinematics)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS roscpp roslib rospy sensor_msgs std_msgs tf
)

## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_node src/ball_kinematics_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node   ${catkin_LIBRARIES} )

add_executable(ball_viz_node src/ball_viz_node.cpp)
add_dependencies(ball_viz_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ball_viz_node ${catkin_LIBRARIES} ${OpenCV_LIBS}) 


