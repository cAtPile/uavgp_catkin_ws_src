cmake_minimum_required(VERSION 3.0.2)
project(mission_master_pkg)

# 编译为 C++11 标准
add_compile_options(-std=c++11)

# 查找依赖包（必须包含消息生成相关组件）
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  mavros_msgs
  actionlib
  actionlib_msgs # Action 消息依赖
  eigen_conversions
  message_generation # 消息生成工具（已包含，无需修改）
)

# 系统依赖（Eigen）
find_package(Eigen3 REQUIRED)

# 声明 Action 消息文件
add_action_files(
  DIRECTORY action
  FILES
  Avoid.action
  Grip.action
)

# 声明自定义消息文件
add_message_files(
  DIRECTORY msg  # 消息文件存放目录
  FILES
  CamTrack.msg  
)

# 生成消息（包含Action和自定义消息，依赖保持不变）
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)

# 声明包信息
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mission_master_lib
  CATKIN_DEPENDS
  roscpp
  rospy
  geometry_msgs
  mavros_msgs
  actionlib
  actionlib_msgs
  eigen_conversions
  message_runtime # 消息运行时依赖（关键）
  DEPENDS EIGEN3
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 声明库文件
add_library(mission_master_lib
  lib/mission_master.cpp
  lib/missionExecute.cpp
  lib/takeoff_kit.cpp
  lib/pickup_kit.cpp
  lib/avoid_kit.cpp
  lib/trace_kit.cpp
  lib/land_kit.cpp
  lib/utils.cpp
  lib/pidctrl.cpp
)

# 库依赖（自动依赖生成的消息目标）
add_dependencies(mission_master_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}  # 包含新增的CamTrack.msg生成的目标
  ${catkin_EXPORTED_TARGETS}
)

# 链接库
target_link_libraries(mission_master_lib
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# 声明可执行文件
add_executable(uavgp25_mission_node
  node/uavgp25_mission_node.cpp
)

# 可执行文件依赖
add_dependencies(uavgp25_mission_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# 链接可执行文件
target_link_libraries(uavgp25_mission_node
  mission_master_lib
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# 安装规则
install(TARGETS mission_master_lib uavgp25_mission_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# 安装Action和消息定义文件（新增msg目录安装）
install(DIRECTORY action/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/action
  FILES_MATCHING PATTERN "*.action"
)
install(DIRECTORY msg/  # 新增：安装msg目录下的消息定义文件
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
  FILES_MATCHING PATTERN "*.msg"
)