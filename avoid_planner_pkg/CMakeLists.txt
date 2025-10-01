cmake_minimum_required(VERSION 3.0.2)
project(avoid_planner_pkg)

add_compile_options(-std=c++14)

# 1. 查找依赖（包含 message_generation，用于生成消息）
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs  
  pcl_ros        
  message_generation  # 消息生成依赖，必须包含
)
find_package(Eigen3 REQUIRED)  # Eigen3 需在 OpenCV 前
find_package(OpenCV REQUIRED)

# 2. 【关键修正1】声明消息文件 + 生成消息（必须在 catkin_package() 之前）
add_message_files(
  FILES
  PolarHistogramMsg.msg  # 你的消息文件，确保在 msg/ 目录下
)

# 生成消息：需指定依赖的其他消息包（如 std_msgs，因你的消息用了 Header）
generate_messages(
  DEPENDENCIES
  std_msgs        # 必须添加，否则 Header 类型无法识别
  geometry_msgs   # 若消息依赖其他包，也需在此添加
)

# 3. 声明 catkin 包信息（此时消息配置已完成，可正确导出）
catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
  LIBRARIES avoid_planner_lib
  # 【关键修正2】依赖 message_runtime（消息运行时，与 message_generation 对应）
  CATKIN_DEPENDS roscpp rospy geometry_msgs pcl_ros message_runtime
  DEPENDS EIGEN3 OpenCV PCL  # 非 catkin 依赖
)

# 4. 包含头文件（添加消息头文件生成路径兜底）
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # 【关键修正3】手动添加消息头文件路径（避免 catkin_INCLUDE_DIRS 未自动包含）
  ${CATKIN_DEVEL_PREFIX}/include  # 等价于 ~/catkin_ws/devel/include，自动适配路径
  ${EIGEN3_INCLUDE_DIRS}
  /usr/include/eigen3          # Eigen 兜底路径
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# 5. 编译自定义库（若库中用到消息，需依赖消息生成目标）
add_library(avoid_planner_lib
  src/lib/cost_calculator.cpp
  src/lib/local_planner.cpp
  src/lib/mavros_interface.cpp
  src/lib/pointcloude_processor.cpp  # 注意：若实际文件名是 pointcloud_processor.cpp，需修正拼写
  src/lib/star_planner.cpp
  src/lib/utils.cpp
)
# 【关键修正4】库依赖消息生成目标（若库中用了 PolarHistogramMsg）
add_dependencies(avoid_planner_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# 链接库（顺序：Eigen → OpenCV → PCL → catkin）
target_link_libraries(avoid_planner_lib
  ${EIGEN3_LIBS}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

# 6. 编译测试节点（核心：确保节点在消息生成后编译）
add_executable(pcp_test_node
  src/test/pcp_test.cpp
)
# 【关键修正5】节点依赖消息生成目标（必须添加，否则编译顺序错误）
add_dependencies(pcp_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# 链接库
target_link_libraries(pcp_test_node
  avoid_planner_lib
  ${EIGEN3_LIBS}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)