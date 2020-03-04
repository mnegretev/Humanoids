cmake_minimum_required(VERSION 2.8.3)
project(ball_position)

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

add_executable(${PROJECT_NAME}_node src/ball_position_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node   ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(compres_ball_viz_node src/compres_ball_viz_node.cpp)
add_dependencies(compres_ball_viz_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(compres_ball_viz_node   ${catkin_LIBRARIES} ${OpenCV_LIBS})
