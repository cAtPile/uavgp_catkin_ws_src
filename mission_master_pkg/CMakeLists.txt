cmake_minimum_required(VERSION 3.0.2)
project(mission_master_pkg)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  mavros_msgs
  actionlib
  eigen_conversions
  message_generation  # 新增：支持自定义Action消息生成
)

## System dependencies
find_package(Eigen3 REQUIRED)

## 生成自定义Action消息（根据你的代码中使用的Action类型）
add_action_files(
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
    eigen_conversions
    message_runtime  # 新增：对应message_generation的运行时依赖
  DEPENDS EIGEN3
)

## Specify include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare C++ library（包含所有实现文件）
add_library(mission_master_lib
  lib/callback.cpp
  lib/mission_master_node.cpp
  lib/client.cpp
  lib/pickAct.cpp    # 拾取动作实现
  lib/avoidAct.cpp   # 避障动作实现
  lib/traceAct.cpp   # 追踪动作实现
)

## Add dependencies to the library（确保消息生成先于编译）
add_dependencies(mission_master_lib 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

## Link libraries
target_link_libraries(mission_master_lib
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

## Declare executable
add_executable(mission_master_node 
  test/test_mission_master_node.cpp
)

## Set output name
set_target_properties(mission_master_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add dependencies to the executable
add_dependencies(mission_master_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

## Link executable to libraries
target_link_libraries(mission_master_node
  mission_master_lib
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

## Install targets
install(TARGETS mission_master_lib mission_master_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Install configuration files
install(FILES
  config/params.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)