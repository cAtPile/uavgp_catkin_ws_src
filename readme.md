er_pkg/mission_master.h:13,
                 from /home/a/catkin/src/mission_master_pkg/lib/pickAct.cpp:5:
/home/a/catkin/src/mission_master_pkg/lib/pickAct.cpp: In member function ‘void MissionMaster::pickFeedbackCB(const PickFeedbackConstPtr&)’:
/home/a/catkin/src/mission_master_pkg/lib/pickAct.cpp:41:49: error: ‘const struct mission_master_pkg::PickFeedback_<std::allocator<void> >’ has no member named ‘pick_current_step’
   41 |     ROS_INFO("Pick current step: %d", feedback->pick_current_step);
      |                                                 ^~~~~~~~~~~~~~~~~
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
/home/a/catkin/src/mission_master_pkg/lib/pickAct.cpp:41:5: note: in expansion of macro ‘ROS_INFO’
   41 |     ROS_INFO("Pick current step: %d", feedback->pick_current_step);
      |     ^~~~~~~~
/home/a/catkin/src/mission_master_pkg/lib/pickAct.cpp:44:22: error: ‘const struct mission_master_pkg::PickFeedback_<std::allocator<void> >’ has no member named ‘pick_current_step’
   44 |     switch(feedback->pick_current_step) {
      |                      ^~~~~~~~~~~~~~~~~
In file included from /opt/ros/noetic/include/ros/ros.h:40,
                 from /home/a/catkin/src/mission_master_pkg/include/mission_master_pkg/mission_master.h:13,
                 from /home/a/catkin/src/mission_master_pkg/lib/pickAct.cpp:5:
/home/a/catkin/src/mission_master_pkg/lib/pickAct.cpp:58:64: error: ‘const struct mission_master_pkg::PickFeedback_<std::allocator<void> >’ has no member named ‘pick_current_step’
   58 |             ROS_INFO("Pick proceeding with step %d", feedback->pick_current_step);
      |                                                                ^~~~~~~~~~~~~~~~~
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
/home/a/catkin/src/mission_master_pkg/lib/pickAct.cpp:58:13: note: in expansion of macro ‘ROS_INFO’
   58 |             ROS_INFO("Pick proceeding with step %d", feedback->pick_current_step);
      |             ^~~~~~~~
/home/a/catkin/src/mission_master_pkg/lib/pickAct.cpp: In member function ‘bool MissionMaster::pickExecute()’:
/home/a/catkin/src/mission_master_pkg/lib/pickAct.cpp:85:34: error: ‘class actionlib::SimpleClientGoalState’ has no member named ‘isActive’
   85 |     if (pick_clientor.getState().isActive()) {
      |                                  ^~~~~~~~
make[2]: *** [mission_master_pkg/CMakeFiles/mission_master_lib.dir/build.make:89: mission_master_pkg/CMakeFiles/mission_master_lib.dir/lib/pickAct.cpp.o] Error 1
make[2]: *** Waiting for unfinished jobs....
[ 94%] Building CXX object mission_master_pkg/CMakeFiles/mission_master_lib.dir/lib/avoidAct.cpp.o
/home/a/catkin/src/mission_master_pkg/lib/avoidAct.cpp: In member function ‘bool MissionMaster::avoidExecute()’:
/home/a/catkin/src/mission_master_pkg/lib/avoidAct.cpp:85:35: error: ‘class actionlib::SimpleClientGoalState’ has no member named ‘isActive’
   85 |     if (avoid_clientor.getState().isActive()) {
      |                                   ^~~~~~~~
make[2]: *** [mission_master_pkg/CMakeFiles/mission_master_lib.dir/build.make:102: mission_master_pkg/CMakeFiles/mission_master_lib.dir/lib/avoidAct.cpp.o] Error 1
make[1]: *** [CMakeFiles/Makefile2:1391: mission_master_pkg/CMakeFiles/mission_master_lib.dir/all] Error 2
make: *** [Makefile:141: all] Error 2
Invoking "make -j2 -l2" failed
a@ubuntu:~/catkin$ 


