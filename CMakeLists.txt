cmake_minimum_required(VERSION 2.8.12)

project(eurecar_local_waypoint_tracker)

#find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  ackermann_msgs
  tf
  cv_bridge  
)

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")
#set(CMAKE_CXX_FLAGS "-W -Wall -O3 -std=c++1z -DNDEBUG")

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED QUIET)

catkin_package(INCLUDE_DIRS include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(
  eurecar_local_waypoint_tracker
  src/eurecar_local_waypoint_tracker.cpp
)
target_link_libraries(eurecar_local_waypoint_tracker
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

add_executable(eurecar_local_waypoint_tracker_node 
src/eurecar_local_waypoint_tracker_node.cpp)

add_dependencies(eurecar_local_waypoint_tracker_node 
   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
   ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(eurecar_local_waypoint_tracker_node
   eurecar_local_waypoint_tracker
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

install(
  TARGETS 
  eurecar_local_waypoint_tracker_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

