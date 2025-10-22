cmake_minimum_required(VERSION 3.0.2)
project(avoid_planner_pkg)

add_compile_options(-std=c++14)

# 查找依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs  
  pcl_ros        
  message_generation  # 消息生成依赖
  actionlib        
  actionlib_msgs   
)
find_package(Eigen3 REQUIRED)  
find_package(OpenCV REQUIRED)

# 声明消息文件 + 生成消息
add_message_files(
  FILES
  PolarFieldMsg.msg  
)

# 声明Action文件
add_action_files(
  FILES
  ApocAvoid.action
)

# 生成消息
generate_messages(
  DEPENDENCIES
  std_msgs        
  geometry_msgs   
  actionlib_msgs
)


#  catkin 包信息
catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
  LIBRARIES avoid_planner_lib
  CATKIN_DEPENDS roscpp rospy geometry_msgs pcl_ros message_runtime
  DEPENDS EIGEN3 OpenCV PCL  # 非 catkin 依赖
)

# 4. 包含头文件
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/include  
  ${EIGEN3_INCLUDE_DIRS}
  /usr/include/eigen3
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# 5. 编译自定义库
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

add_dependencies(avoid_planner_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# 链接库
target_link_libraries(avoid_planner_lib
  ${EIGEN3_LIBS}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

# 6. 编译测试节点
add_executable(pcp_test_node
  test/test.cpp
) 

#
add_dependencies(pcp_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# 链接库
target_link_libraries(pcp_test_node
  avoid_planner_lib
  ${EIGEN3_LIBS}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
