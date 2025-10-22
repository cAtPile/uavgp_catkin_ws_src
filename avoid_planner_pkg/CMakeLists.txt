cmake_minimum_required(VERSION 3.0.2)
project(avoid_planner_pkg)

# 设置C++14标准
add_compile_options(-std=c++14)

# 查找依赖项
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs  
  pcl_ros        
  message_generation  # 消息生成依赖
  actionlib        
  actionlib_msgs   
)

# 查找Eigen3和OpenCV库
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# 查找PCL库
find_package(PCL REQUIRED)

# 声明消息文件 + 生成消息
add_message_files(
  FILES
  PolarFieldMsg.msg  # 你自己的消息文件
)

# 声明Action文件
add_action_files(
  FILES
  ApocAvoid.action  # 你自己的Action文件
)

# 生成消息
generate_messages(
  DEPENDENCIES
  std_msgs        
  geometry_msgs   
  actionlib_msgs
)

# 声明Catkin包
catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
  LIBRARIES avoid_planner_lib
  CATKIN_DEPENDS roscpp rospy geometry_msgs pcl_ros message_runtime
  DEPENDS EIGEN3 OpenCV PCL  # 非catkin依赖
)

# 头文件包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include  # 确保include目录被正确加入
  ${PROJECT_SOURCE_DIR}/action  # 添加action目录
)

# 编译自定义库
add_library(avoid_planner_lib
  lib/anglebinIndex.cpp
  lib/avoid_planner.cpp
  lib/calculateAtt.cpp
  lib/calculateRep.cpp
  lib/calculateTotal.cpp
  lib/filterPC.cpp
  lib/generateForceDir.cpp
  lib/generatePFdismap.cpp
  lib/generatePFpotmap.cpp
  lib/goalCB.cpp
  lib/pointcloudCB.cpp
  lib/tfBodyframe.cpp
  lib/updatePFpoint.cpp
)

# 设置依赖关系，确保库被正确编译
add_dependencies(avoid_planner_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 链接库
target_link_libraries(avoid_planner_lib
  ${EIGEN3_LIBS}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

# 编译测试节点
add_executable(pcp_test_node
  test/test.cpp
)

# 设置依赖关系，确保节点正确编译
add_dependencies(pcp_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 链接库
target_link_libraries(pcp_test_node
  avoid_planner_lib
  ${EIGEN3_LIBS}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

# 安装设置 (如果需要的话)
install(TARGETS avoid_planner_lib pcp_test_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# 安装消息文件
install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
)

# 安装action文件
install(DIRECTORY action/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/action
)

