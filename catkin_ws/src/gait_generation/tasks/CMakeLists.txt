cmake_minimum_required(VERSION 2.8.3)
project(tasks)

find_package(catkin REQUIRED COMPONENTS
  humanoid
  humanoid_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tasks
#  CATKIN_DEPENDS humanoid humanoid_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(walk_and_kick src/walk_and_kick.cpp)
add_executable(kick_and_walk src/kick_and_walk.cpp)

add_dependencies(walk_and_kick ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(kick_and_walk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(walk_and_kick ${catkin_LIBRARIES})
target_link_libraries(kick_and_walk ${catkin_LIBRARIES})
