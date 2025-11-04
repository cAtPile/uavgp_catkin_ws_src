cmake_minimum_required(VERSION 3.0.2)
project(mission_master_pkg)

## 编译为 C++11 标准
add_compile_options(-std=c++11)

## 查找依赖包（必须包含消息生成相关组件）
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  mavros_msgs
  actionlib
  actionlib_msgs  # Action 消息依赖
  eigen_conversions
  message_generation  # 消息生成工具
)

## 系统依赖（Eigen）
find_package(Eigen3 REQUIRED)

## 声明 Action 消息文件（关键步骤：指定目录和文件）
add_action_files(
  DIRECTORY action  # 明确 Action 文件所在目录（必须存在）
  FILES
  AvoidAction.action
  PickAction.action
  TraceAction.action
)

## 生成消息（依赖 actionlib_msgs 等基础消息）
generate_messages(
  DEPENDENCIES
  actionlib_msgs  # 必须包含，否则无法生成 Action 头文件
  std_msgs
  geometry_msgs
)

## 声明包信息（包含消息运行时依赖）
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mission_master_lib
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    geometry_msgs 
    mavros_msgs 
    actionlib 
    actionlib_msgs  # 必须包含
    eigen_conversions
    message_runtime  # 消息运行时依赖（关键）
  DEPENDS EIGEN3
)

## 包含目录（确保能找到生成的头文件）
include_directories(
  include
  ${catkin_INCLUDE_DIRS}  # 自动包含 devel/include 目录（生成的头文件在这里）
  ${EIGEN3_INCLUDE_DIRS}
)

## 声明库文件（包含所有源文件）
add_library(mission_master_lib
  lib/callback.cpp
  lib/mission_master_node.cpp
  lib/pickAct.cpp
  lib/avoidAct.cpp
  lib/traceAct.cpp
)

## 库依赖（必须依赖自动生成的消息目标）
add_dependencies(mission_master_lib 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}  # 本包生成的消息目标（关键）
  ${catkin_EXPORTED_TARGETS}
)

## 链接库
target_link_libraries(mission_master_lib
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

## 声明可执行文件
add_executable(mission_master_node 
  test/test_mission_master_node.cpp
)

## 设置可执行文件名称
set_target_properties(mission_master_node PROPERTIES OUTPUT_NAME node PREFIX "")

## 可执行文件依赖（同样依赖消息目标）
add_dependencies(mission_master_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}  # 关键：确保消息生成后再编译本节点
  ${catkin_EXPORTED_TARGETS}
)

## 链接可执行文件
target_link_libraries(mission_master_node
  mission_master_lib
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

## 安装规则
install(TARGETS mission_master_lib mission_master_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## 安装 Action 定义文件（确保其他包能找到）
install(DIRECTORY action/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/action
  FILES_MATCHING PATTERN "*.action"
)