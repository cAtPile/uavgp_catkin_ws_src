### 文件结构

### save
---
^Cheader: 
  seq: 114
  stamp: 
    secs: 1761660783
    nsecs:  95768806
  frame_id: "base_link"
azimuth_resolution: 0.17453292519943295
elevation_resolution: 0.08726646259971647
min_azimuth: -3.141592653589793
max_azimuth: 3.141592653589793
min_elevation: 0.0
max_elevation: 0.8726646259971648
max_range: 2.0
min_range: 0.0
num_azimuth_bins: 37
num_elevation_bins: 11
obstacle_distances: []
pot_map: []
force_vector: 
  x: 0.0
  y: 0.0
  z: 0.0
local_position: 
  x: 0.0
  y: 0.0
  z: 0.0
---
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


