cmake_minimum_required(VERSION 3.0.2)
project(avoid_planner_pkg)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs  
  pcl_ros        
)

find_package(Eigen3 REQUIRED)  
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS} 
  LIBRARIES avoid_planner_lib
  CATKIN_DEPENDS roscpp rospy
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} 
  ${OpenCV_LIBRARIES}
)

add_library(avoid_planner_lib
  src/lib/cost_calculator.cpp
  src/lib/local_planner.cpp
  src/lib/mavros_interface.cpp
  src/lib/pointcloude_processor.cpp
  src/lib/star_planner.cpp
  src/lib/utils.cpp
)

target_link_libraries(avoid_planner_lib
  ${catkin_LIBRARIES}
)

add_executable(pcp_test_node
  src/test/pc_test.cpp
)

target_link_libraries(pcp_test_node
  avoid_planner_lib
  ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)