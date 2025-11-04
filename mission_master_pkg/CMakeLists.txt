cmake_minimum_required(VERSION 3.0.2)
project(mission_master_pkg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  mavros_msgs
  actionlib
  actionlib_msgs  # Add this line
  eigen_conversions
  message_generation
)

find_package(Eigen3 REQUIRED)

# Generate action messages
add_action_files(
  DIRECTORY action  # Specify directory explicitly
  FILES
  AvoidAction.action
  PickAction.action
  TraceAction.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mission_master_lib
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    geometry_msgs 
    mavros_msgs 
    actionlib 
    actionlib_msgs  # Add this
    eigen_conversions
    message_runtime
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(mission_master_lib
  lib/callback.cpp
  lib/mission_master_node.cpp
  lib/pickAct.cpp
  lib/avoidAct.cpp
  lib/traceAct.cpp
)

add_dependencies(mission_master_lib 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(mission_master_lib
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(mission_master_node 
  test/test_mission_master_node.cpp
)

set_target_properties(mission_master_node PROPERTIES OUTPUT_NAME node PREFIX "")

add_dependencies(mission_master_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(mission_master_node
  mission_master_lib
  ${catkin_LIBRARIES}
)

install(TARGETS mission_master_lib mission_master_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)