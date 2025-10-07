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
)
find_package(Eigen3 REQUIRED)  
find_package(OpenCV REQUIRED)

# 声明消息文件 + 生成消息
add_message_files(
  FILES
  PolarHistogramMsg.msg  
)

# 生成消息
generate_messages(
  DEPENDENCIES
  std_msgs        
  geometry_msgs   
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
  src/lib/cost_calculator.cpp
  src/lib/local_planner.cpp
  src/lib/mavros_interface.cpp
  src/lib/star_planner.cpp
  src/lib/utils.cpp
  lib/pointcloud_processor/angle_tobin_index.cpp
  lib/pointcloud_processor/filter_pointcloud.cpp
  lib/pointcloud_processor/generate_polarhistogram.cpp
  lib/pointcloud_processor/get_angle_bin_index.cpp
  lib/pointcloud_processor/get_histogram.cpp
  lib/pointcloud_processor/is_updated.cpp
  lib/pointcloud_processor/load_params.cpp
  lib/pointcloud_processor/pointcloud_processor.cpp
  lib/pointcloud_processor/pointcloude_callback.cpp
  lib/pointcloud_processor/reset_updated_flag.cpp
  lib/pointcloud_processor/transform_to_bodyframe.cpp
  lib/pointcloud_processor/update_histogram_from_point.cpp
  lib/potential_field/calculate_repulsive_force.cpp
  lib/potential_field/calculate_total_force.cpp
  lib/potential_field/generate_potentialfield.cpp
  lib/potential_field/generate_total_force.cpp
  lib/potential_field/get_polar_histogram.cpp
  lib/potential_field/load_params.cpp
  lib/potential_field/potential_field.cpp
  lib/potential_field/set.cpp
  lib/potential_field/update_polar_goal.cpp
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
  test/pcp_test.cpp
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

add_executable(pf_test_node
  test/pf_test_node.cpp
)

target_link_libraries(pf_test_node
  avoid_planner_lib
  ${EIGEN3_LIBS}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
