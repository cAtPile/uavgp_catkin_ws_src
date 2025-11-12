cmake_minimum_required(VERSION 3.0.2)
project(mission_master_pkg)

# # 编译为 C++11 标准
add_compile_options(-std=c++11)

# # 查找依赖包（必须包含消息生成相关组件）
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  mavros_msgs
  actionlib
  actionlib_msgs # Action 消息依赖
  eigen_conversions
  message_generation # 消息生成工具
)

# # 系统依赖（Eigen）
find_package(Eigen3 REQUIRED)

# # 声明 Action 消息文件（关键步骤：指定目录和文件）
add_action_files(
  DIRECTORY action # 明确 Action 文件所在目录（必须存在）
  FILES
  Avoid.action
  Pick.action
  Trace.action
)

# # 生成消息（依赖 actionlib_msgs 等基础消息）
generate_messages(
  DEPENDENCIES
  actionlib_msgs # 必须包含，否则无法生成 Action 头文件
  std_msgs
  geometry_msgs
)

# # 声明包信息（包含消息运行时依赖）
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mission_master_lib
  CATKIN_DEPENDS
  roscpp
  rospy
  geometry_msgs
  mavros_msgs
  actionlib
  actionlib_msgs # 必须包含
  eigen_conversions
  message_runtime # 消息运行时依赖（关键）
  DEPENDS EIGEN3
)

# # 包含目录（确保能找到生成的头文件）
include_directories(
  include
  ${catkin_INCLUDE_DIRS} # 自动包含 devel/include 目录（生成的头文件在这里）
  ${EIGEN3_INCLUDE_DIRS}
)

# # 声明库文件（包含所有源文件，补充遗漏的功能模块）
add_library(mission_master_lib
  lib/mission_master.cpp # 核心类实现
  lib/missionExecute.cpp # 任务主循环
  lib/takeoff_kit.cpp # 起飞相关
  lib/pickup_kit.cpp # 抓取相关
  lib/avoid_kit.cpp # 避障相关
  lib/trace_kit.cpp # 跟踪相关
  lib/land_kit.cpp # 降落相关
  lib/utils.cpp # 工具函数（定点、到达检查）
)

# # 库依赖（必须依赖自动生成的消息目标）
add_dependencies(mission_master_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS} # 本包生成的消息目标（关键）
  ${catkin_EXPORTED_TARGETS}
)

# # 链接库
target_link_libraries(mission_master_lib
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# # 声明可执行文件（包含测试节点和实际任务节点）
# 测试节点

# 实际任务节点
add_executable(uavgp25_mission_node
  node/uavgp25_mission_node.cpp
)

# # 可执行文件依赖
add_dependencies(uavgp25_mission_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# # 链接可执行文件
target_link_libraries(uavgp25_mission_node
  mission_master_lib
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# # 安装规则
install(TARGETS mission_master_lib uavgp25_mission_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# # 安装 Action 定义文件
install(DIRECTORY action/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/action
  FILES_MATCHING PATTERN "*.action"
)