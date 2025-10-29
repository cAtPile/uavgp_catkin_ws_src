### 文件结构

### save
 84%] Building CXX object avoid_planner_pkg/CMakeFiles/avoid_planner_lib.dir/lib/generatePFpotmap.cpp.o
/home/a/catkin_ws/src/avoid_planner_pkg/lib/generatePFpotmap.cpp: In member function ‘void AvoidPlanner::generatePFpotmap(double, double, double)’:
/home/a/catkin_ws/src/avoid_planner_pkg/lib/generatePFpotmap.cpp:45:17: warning: init-statement in selection statements only available with ‘-std=c++17’ or ‘-std=gnu++17’
   45 |             if (az_idx == static_cast<size_t>(goal_az_idx) && el_idx == static_cast<size_t>(goal_el_idx) {
      |                 ^~~~~~
/home/a/catkin_ws/src/avoid_planner_pkg/lib/generatePFpotmap.cpp:45:105: error: expected ‘;’ before ‘{’ token
   45 |             if (az_idx == static_cast<size_t>(goal_az_idx) && el_idx == static_cast<size_t>(goal_el_idx) {
      |                                                                                                         ^~
      |                                                                                                         ;
/home/a/catkin_ws/src/avoid_planner_pkg/lib/generatePFpotmap.cpp:50:66: error: expected ‘)’ before ‘;’ token
   50 |             double repulsive_force = calculateRep(az_idx, el_idx);
      |                                                                  ^
      |                                                                  )
/home/a/catkin_ws/src/avoid_planner_pkg/lib/generatePFpotmap.cpp:45:16: note: to match this ‘(’
   45 |             if (az_idx == static_cast<size_t>(goal_az_idx) && el_idx == static_cast<size_t>(goal_el_idx) {
      |                ^
/home/a/catkin_ws/src/avoid_planner_pkg/lib/generatePFpotmap.cpp:53:62: error: ‘repulsive_force’ was not declared in this scope
   53 |             calculateTotal(az_idx, el_idx, attractive_force, repulsive_force);
      |                                                              ^~~~~~~~~~~~~~~
make[2]: *** [avoid_planner_pkg/CMakeFiles/avoid_planner_lib.dir/build.make:167: avoid_planner_pkg/CMakeFiles/avoid_planner_lib.dir/lib/generatePFpotmap.cpp.o] Error 1
make[1]: *** [CMakeFiles/Makefile2:2150: avoid_planner_pkg/CMakeFiles/avoid_planner_lib.dir/all] Error 2
make: *** [Makefile:141: all] Error 2
Invoking "make -j6 -l6" failed
a@ubuntu:~/catkin_ws$ 



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


