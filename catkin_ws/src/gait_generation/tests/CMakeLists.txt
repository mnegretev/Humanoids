cmake_minimum_required(VERSION 2.8.3)
project(tests)

find_package(catkin REQUIRED COMPONENTS
  humanoid
  humanoid_msgs
  roscpp
  rospy
  std_msgs
  ctrl_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tests
#  CATKIN_DEPENDS humanoid humanooid_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


# add_executable(walk src/walk.cpp)
# add_executable(left_kick src/left_kick.cpp)
# add_executable(right_kick src/right_kick.cpp)
# add_executable(prone_get_up src/prone_get_up.cpp)
# add_executable(supine_get_up src/supine_get_up.cpp)
# add_executable(ready_to_kick src/ready_to_kick.cpp)
# 
# 
# add_dependencies(walk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(left_kick  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(right_kick ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(prone_get_up  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(supine_get_up ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(ready_to_kick ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# 
# 
# target_link_libraries(walk ${catkin_LIBRARIES})
# target_link_libraries(left_kick  ${catkin_LIBRARIES})
# target_link_libraries(right_kick ${catkin_LIBRARIES})
# target_link_libraries(prone_get_up  ${catkin_LIBRARIES})
# target_link_libraries(supine_get_up ${catkin_LIBRARIES})
# target_link_libraries(ready_to_kick ${catkin_LIBRARIES})
