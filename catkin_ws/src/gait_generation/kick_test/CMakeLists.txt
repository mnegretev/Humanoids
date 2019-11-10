cmake_minimum_required(VERSION 2.8.3)
project(kick_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

add_service_files(
  FILES
  getPose.srv
  speedProfile.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs sensor_msgs nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/kick_test_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
