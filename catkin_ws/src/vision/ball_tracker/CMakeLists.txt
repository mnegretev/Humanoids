cmake_minimum_required(VERSION 2.8.3)
project(ball_tracker)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
  tf
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ball_tracker
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/ball_tracker_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(ball_tracker_simul src/ball_tracker_simul.cpp)
add_dependencies(ball_tracker_simul ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ball_tracker_simul ${catkin_LIBRARIES} ${OpenCV_LIBS})

