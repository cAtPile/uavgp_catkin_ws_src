### 文件结构

### save
    livox_lidar_publisher2 (livox_ros_driver2/livox_ros_driver2_node)

auto-starting new master
process[master]: started with pid [21167]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to cf22c648-b340-11f0-8fa9-60ff9e774db9
process[rosout-1]: started with pid [21182]
started core service [/rosout]
process[livox_lidar_publisher2-2]: started with pid [21189]
[INFO] [1761575139.332381382]: Livox Ros Driver2 Version: 1.2.4
data source:0.
[INFO] [1761575139.336441191]: Data Source is raw lidar.
[INFO] [1761575139.336811123]: Config file : /home/a/ws_livox/src/livox_ros_driver2/config/MID360_config.json
LdsLidar *GetInstance
config lidar type: 8
successfully parse base config, counts: 1
[2025-10-27 22:25:39.337] [console] [info] set master/slave sdk to master sdk by default  [parse_cfg_file.cpp] [Parse] [82]
[2025-10-27 22:25:39.337] [console] [info] Livox lidar logger disable.  [parse_cfg_file.cpp] [Parse] [126]
[2025-10-27 22:25:39.337] [console] [info] Device type:9 point cloud data and IMU data unicast is enabled.  [params_check.cpp] [CheckLidarMulticastIp] [100]
[2025-10-27 22:25:39.337] [console] [info] Data Handler Init Succ.  [data_handler.cpp] [Init] [49]
bind failed
[2025-10-27 22:25:39.338] [console] [error] Create detection socket failed.  [device_manager.cpp] [CreateDetectionChannel] [275]
[2025-10-27 22:25:39.338] [console] [error] Create detection channel failed.  [device_manager.cpp] [CreateChannel] [242]
[2025-10-27 22:25:39.338] [console] [error] Create channel failed.  [device_manager.cpp] [Init] [169]
Failed to init livox lidar sdk.
[ERROR] [1761575139.338086907]: Init lds lidar failed!
^C[livox_lidar_publisher2-2] killing on exit
^C^C[livox_lidar_publisher2-2] escalating to SIGTERM
^C[rosout-1] killing on exit
^C^C[master] killing on exit
shutting down processing monitor...
... shutting down processing monitor complete
done
a@ubuntu:~/ws_livox$ ^C



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


