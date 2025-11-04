lib/mission_master.cpp.o
/home/a/catkin/src/mission_master_pkg/lib/callback.cpp: In member function ‘void MissionMaster::localPoseCB(const ConstPtr&)’:
/home/a/catkin/src/mission_master_pkg/lib/callback.cpp:10:20: error: no match for ‘operator=’ (operand types are ‘geometry_msgs::PoseStamped’ {aka ‘geometry_msgs::PoseStamped_<std::allocator<void> >’} and ‘const ConstPtr’ {aka ‘const boost::shared_ptr<const geometry_msgs::PoseStamped_<std::allocator<void> > >’})
   10 |     current_pose = msg;
      |                    ^~~
In file included from /home/a/catkin/src/mission_master_pkg/include/mission_master_pkg/mission_master.h:14,
                 from /home/a/catkin/src/mission_master_pkg/lib/callback.cpp:6:
/opt/ros/noetic/include/geometry_msgs/PoseStamped.h:24:8: note: candidate: ‘geometry_msgs::PoseStamped_<std::allocator<void> >& geometry_msgs::PoseStamped_<std::allocator<void> >::operator=(const geometry_msgs::PoseStamped_<std::allocator<void> >&)’
   24 | struct PoseStamped_
      |        ^~~~~~~~~~~~
/opt/ros/noetic/include/geometry_msgs/PoseStamped.h:24:8: note:   no known conversion for argument 1 from ‘const ConstPtr’ {aka ‘const boost::shared_ptr<const geometry_msgs::PoseStamped_<std::allocator<void> > >’} to ‘const geometry_msgs::PoseStamped_<std::allocator<void> >&’
/opt/ros/noetic/include/geometry_msgs/PoseStamped.h:24:8: note: candidate: ‘geometry_msgs::PoseStamped_<std::allocator<void> >& geometry_msgs::PoseStamped_<std::allocator<void> >::operator=(geometry_msgs::PoseStamped_<std::allocator<void> >&&)’
/opt/ros/noetic/include/geometry_msgs/PoseStamped.h:24:8: note:   no known conversion for argument 1 from ‘const ConstPtr’ {aka ‘const boost::shared_ptr<const geometry_msgs::PoseStamped_<std::allocator<void> > >’} to ‘geometry_msgs::PoseStamped_<std::allocator<void> >&&’
/home/a/catkin/src/mission_master_pkg/lib/callback.cpp: In member function ‘void MissionMaster::stateCheckCB(const ConstPtr&)’:
/home/a/catkin/src/mission_master_pkg/lib/callback.cpp:17:30: error: no match for ‘operator=’ (operand types are ‘mavros_msgs::State’ {aka ‘mavros_msgs::State_<std::allocator<void> >’} and ‘const ConstPtr’ {aka ‘const boost::shared_ptr<const mavros_msgs::State_<std::allocator<void> > >’})
   17 |     current_vehicle_state_ = msg;
      |                              ^~~
In file included from /home/a/catkin/src/mission_master_pkg/include/mission_master_pkg/mission_master.h:15,
                 from /home/a/catkin/src/mission_master_pkg/lib/callback.cpp:6:
/opt/ros/noetic/include/mavros_msgs/State.h:23:8: note: candidate: ‘mavros_msgs::State_<std::allocator<void> >& mavros_msgs::State_<std::allocator<void> >::operator=(const mavros_msgs::State_<std::allocator<void> >&)’
   23 | struct State_
      |        ^~~~~~
/opt/ros/noetic/include/mavros_msgs/State.h:23:8: note:   no known conversion for argument 1 from ‘const ConstPtr’ {aka ‘const boost::shared_ptr<const mavros_msgs::State_<std::allocator<void> > >’} to ‘const mavros_msgs::State_<std::allocator<void> >&’
/opt/ros/noetic/include/mavros_msgs/State.h:23:8: note: candidate: ‘mavros_msgs::State_<std::allocator<void> >& mavros_msgs::State_<std::allocator<void> >::operator=(mavros_msgs::State_<std::allocator<void> >&&)’
/opt/ros/noetic/include/mavros_msgs/State.h:23:8: note:   no known conversion for argument 1 from ‘const ConstPtr’ {aka ‘const boost::shared_ptr<const mavros_msgs::State_<std::allocator<void> > >’} to ‘mavros_msgs::State_<std::allocator<void> >&&’
/home/a/catkin/src/mission_master_pkg/lib/callback.cpp:20:41: error: ‘ARM’ was not declared in this scope
   20 |     if (current_vehicle_state_.armed == ARM)
      |                                         ^~~
make[2]: *** [mission_master_pkg/CMakeFiles/mission_master_lib.dir/build.make:63: mission_master_pkg/CMakeFiles/mission_master_lib.dir/lib/callback.cpp.o] Error 1
make[2]: *** Waiting for unfinished jobs....
make[1]: *** [CMakeFiles/Makefile2:1391: mission_master_pkg/CMakeFiles/mission_master_lib.dir/all] Error 2
make: *** [Makefile:141: all] Error 2
Invoking "make -j2 -l2" f
