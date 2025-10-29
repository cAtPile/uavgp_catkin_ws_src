### 文件结构

### save
[ 84%] Built target avoid_planner_pkg_generate_messages_eus
[ 84%] Built target cam_tracker_generate_messages
[ 84%] Built target avoid_planner_pkg_generate_messages
Scanning dependencies of target avoid_planner_lib
[ 86%] Building CXX object avoid_planner_pkg/CMakeFiles/avoid_planner_lib.dir/lib/calculateRep.cpp.o
In file included from /opt/ros/noetic/include/ros/ros.h:40,
                 from /home/a/catkin_ws/src/avoid_planner_pkg/include/avoid_planner_pkg/avoid_planner.h:13,
                 from /home/a/catkin_ws/src/avoid_planner_pkg/lib/calculateRep.cpp:8:
/home/a/catkin_ws/src/avoid_planner_pkg/lib/calculateRep.cpp: In member function ‘double AvoidPlanner::calculateRep(size_t, size_t)’:
/home/a/catkin_ws/src/avoid_planner_pkg/lib/calculateRep.cpp:35:14: warning: format ‘%d’ expects argument of type ‘int’, but argument 8 has type ‘size_t’ {aka ‘long unsigned int’} [-Wformat=]
   35 |     ROS_INFO("azBin: %d ,elBin: %d, obs_dis= %0.2f , rep_force=%0.2f",az_idx,el_idx,rep_force);
      |              ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ~~~~~~
      |                                                                       |
      |                                                                       size_t {aka long unsigned int}
/opt/ros/noetic/include/ros/console.h:351:165: note: in definition of macro ‘ROSCONSOLE_PRINT_AT_LOCATION_WITH_FILTER’
  351 |     ::ros::console::print(filter, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, __FILE__, __LINE__, __ROSCONSOLE_FUNCTION__, __VA_ARGS__)
      |                                                                                                                                                                     ^~~~~~~~~~~
/opt/ros/noetic/include/ros/console.h:390:7: note: in expansion of macro ‘ROSCONSOLE_PRINT_AT_LOCATION’
  390 |       ROSCONSOLE_PRINT_AT_LOCATION(__VA_ARGS__); \
      |       ^~~~~~~~~~~~~~~~~~~~~~~~~~~~
/opt/ros/noetic/include/ros/console.h:575:35: note: in expansion of macro ‘ROS_LOG_COND’
  575 | #define ROS_LOG(level, name, ...) ROS_LOG_COND(true, level, name, __VA_ARGS__)
      |                                   ^~~~~~~~~~~~
/opt/ros/noetic/include/rosconsole/macros_generated.h:110:23: note: in expansion of macro ‘ROS_LOG’
  110 | #define ROS_INFO(...) ROS_LOG(::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
      |                       ^~~~~~~
/home/a/catkin_ws/src/avoid_planner_pkg/lib/calculateRep.cpp:35:5: note: in expansion of macro ‘ROS_INFO’
   35 |     ROS_INFO("azBin: %d ,elBin: %d, obs_dis= %0.2f , rep_force=%0.2f",az_idx,el_idx,rep_force);
      |     ^~~~~~~~
/home/a/catkin_ws/src/avoid_planner_pkg/lib/calculateRep.cpp:35:23: note: format string is defined here
   35 |     ROS_INFO("azBin: %d ,elBin: %d, obs_dis= %0.2f , rep_force=%0.2f",az_idx,el_idx,rep_force);
      |                      ~^
      |                       |
      |                       int
      |                      %ld
In file included from /opt/ros/noetic/include/ros/ros.h:40,
                 from /home/a/catkin_ws/src/avoid_planner_pkg/include/avoid_planner_pkg/avoid_planner.h:13,
                 from /home/a/catkin_ws/src/avoid_planner_pkg/lib/calculateRep.cpp:8:
/home/a/catkin_ws/src/avoid_planner_pkg/lib/calculateRep.cpp:35:14: warning: format ‘%d’ expects argument of type ‘int’, but argument 9 has type ‘size_t’ {aka ‘long unsigned int’} [-Wformat=]
   35 |     ROS_INFO("azBin: %d ,elBin: %d, obs_dis= %0.2f , rep_force=%0.2f",az_idx,el_idx,rep_force);
      |              ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~        ~~~~~~
      |                                                                              |
      |                                                                              size_t {aka long unsigned int}
/opt/ros/noetic/include/ros/console.h:351:165: note: in definition of macro ‘ROSCONSOLE_PRINT_AT_LOCATION_WITH_FILTER’
  351 |     ::ros::console::print(filter, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, __FILE__, __LINE__, __ROSCONSOLE_FUNCTION__, __VA_ARGS__)
      |                                                                                                                                                                     ^~~~~~~~~~~
/opt/ros/noetic/include/ros/console.h:390:7: note: in expansion of macro ‘ROSCONSOLE_PRINT_AT_LOCATION’
  390 |       ROSCONSOLE_PRINT_AT_LOCATION(__VA_ARGS__); \
      |       ^~~~~~~~~~~~~~~~~~~~~~~~~~~~
/opt/ros/noetic/include/ros/console.h:575:35: note: in expansion of macro ‘ROS_LOG_COND’
  575 | #define ROS_LOG(level, name, ...) ROS_LOG_COND(true, level, name, __VA_ARGS__)
      |                                   ^~~~~~~~~~~~
/opt/ros/noetic/include/rosconsole/macros_generated.h:110:23: note: in expansion of macro ‘ROS_LOG’
  110 | #define ROS_INFO(...) ROS_LOG(::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
      |                       ^~~~~~~
/home/a/catkin_ws/src/avoid_planner_pkg/lib/calculateRep.cpp:35:5: note: in expansion of macro ‘ROS_INFO’
   35 |     ROS_INFO("azBin: %d ,elBin: %d, obs_dis= %0.2f , rep_force=%0.2f",az_idx,el_idx,rep_force);
      |     ^~~~~~~~
/home/a/catkin_ws/src/avoid_planner_pkg/lib/calculateRep.cpp:35:34: note: format string is defined here
   35 |     ROS_INFO("azBin: %d ,elBin: %d, obs_dis= %0.2f , rep_force=%0.2f",az_idx,el_idx,rep_force);
      |                                 ~^
      |                                  |
      |                                  int
      |                                 %ld
In file included from /opt/ros/noetic/include/ros/ros.h:40,
                 from /home/a/catkin_ws/src/avoid_planner_pkg/include/avoid_planner_pkg/avoid_planner.h:13,
                 from /home/a/catkin_ws/src/avoid_planner_pkg/lib/calculateRep.cpp:8:
/home/a/catkin_ws/src/avoid_planner_pkg/lib/calculateRep.cpp:35:14: warning: format ‘%f’ expects a matching ‘double’ argument [-Wformat=]
   35 |     ROS_INFO("azBin: %d ,elBin: %d, obs_dis= %0.2f , rep_force=%0.2f",az_idx,el_idx,rep_force);
      |              ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/opt/ros/noetic/include/ros/console.h:351:165: note: in definition of macro ‘ROSCONSOLE_PRINT_AT_LOCATION_WITH_FILTER’
  351 |     ::ros::console::print(filter, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_, __FILE__, __LINE__, __ROSCONSOLE_FUNCTION__, __VA_ARGS__)
      |                                                                                                                                                                     ^~~~~~~~~~~
/opt/ros/noetic/include/ros/console.h:390:7: note: in expansion of macro ‘ROSCONSOLE_PRINT_AT_LOCATION’
  390 |       ROSCONSOLE_PRINT_AT_LOCATION(__VA_ARGS__); \
      |       ^~~~~~~~~~~~~~~~~~~~~~~~~~~~
/opt/ros/noetic/include/ros/console.h:575:35: note: in expansion of macro ‘ROS_LOG_COND’
  575 | #define ROS_LOG(level, name, ...) ROS_LOG_COND(true, level, name, __VA_ARGS__)
      |                                   ^~~~~~~~~~~~
/opt/ros/noetic/include/rosconsole/macros_generated.h:110:23: note: in expansion of macro ‘ROS_LOG’
  110 | #define ROS_INFO(...) ROS_LOG(::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
      |                       ^~~~~~~
/home/a/catkin_ws/src/avoid_planner_pkg/lib/calculateRep.cpp:35:5: note: in expansion of macro ‘ROS_INFO’
   35 |     ROS_INFO("azBin: %d ,elBin: %d, obs_dis= %0.2f , rep_force=%0.2f",az_idx,el_idx,rep_force);
      |     ^~~~~~~~
/home/a/catkin_ws/src/avoid_planner_pkg/lib/calculateRep.cpp:35:68: note: format string is defined here
   35 |     ROS_INFO("azBin: %d ,elBin: %d, obs_dis= %0.2f , rep_force=%0.2f",az_idx,el_idx,rep_force);
      |                                                                ~~~~^
      |                                                                    |
      |                                                                    double
[ 87%] Linking CXX shared library /home/a/catkin_ws/devel/lib/libavoid_planner_lib.so
[ 97%] Built target avoid_planner_lib
[ 98%] Linking CXX executable /home/a/catkin_ws/devel/lib/avoid_planner_pkg/pcp_test_node
[100%] Built target pcp_test_node


待办：
1.构建一个action
获取目标点位置和行动
发送位移方向

2.构建一个
同一的结构体

思路

有一个数组：
obstacle[][]
第一维度是俯仰角，从min_el 到 max_el步长step_el
第二维度是方位角，从min_az 到 max_az步长step_az
存储的数据是距离障碍物的距离
计算斥力
repulsion=repulsion_ration/obstacle_distance
将斥力存储到数组
repulsion[][]
有一个目标点 gaol_el,gaol_az,gaol_dis
计算引力gravity
计算合力
选择方向


