cmake_minimum_required(VERSION 3.0.2)
project(avoid_planner_pkg)

add_compile_options(-std=c++14)

# 1. 查找依赖（Eigen3 必须在 OpenCV 之前）
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs  
  pcl_ros        
)
find_package(Eigen3 REQUIRED)  # 先查找 Eigen3
find_package(OpenCV REQUIRED)  # 后查找 OpenCV
find_package(PCL REQUIRED)     # 若使用 PCL，需添加

# 2. 声明 catkin 包信息
catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
  LIBRARIES avoid_planner_lib
  CATKIN_DEPENDS roscpp rospy geometry_msgs pcl_ros
  DEPENDS EIGEN3 OpenCV PCL  # 声明非 catkin 依赖
)

# 3. 包含头文件（顺序：Eigen → OpenCV → PCL → 自定义）
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}       # 第一优先级：Eigen 头文件
  /usr/include/eigen3          # 兜底：手动指定 Eigen 路径
  ${OpenCV_INCLUDE_DIRS}       # 第二优先级：OpenCV 头文件
  ${PCL_INCLUDE_DIRS}          # 第三优先级：PCL 头文件
)

# 4. 编译自定义库
add_library(avoid_planner_lib
  src/lib/cost_calculator.cpp
  src/lib/local_planner.cpp
  src/lib/mavros_interface.cpp
  src/lib/pointcloude_processor.cpp  # 注意：文件名若为 pointcloud_processor.cpp，需修正
  src/lib/star_planner.cpp
  src/lib/utils.cpp
)
# 链接库（顺序：Eigen → OpenCV → PCL → catkin）
target_link_libraries(avoid_planner_lib
  ${EIGEN3_LIBS}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

# 5. 编译测试节点
add_executable(pcp_test_node
  src/test/pc_test.cpp
)
# 测试节点链接库（与自定义库一致）
target_link_libraries(pcp_test_node
  avoid_planner_lib
  ${EIGEN3_LIBS}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)