# Mission Master - 快速启动指南

## 一、编译

```bash
cd ~/drone_uavgp_ws
catkin build mission_master_pkg
source devel/setup.bash
```

## 二、运行（需要3个终端）

### 终端1: 启动PX4仿真

```bash
cd ~/PX4_Firmware
make px4_sitl gazebo
```

### 终端2: 启动MAVROS

```bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

### 终端3: 启动Mission Master

```bash
cd ~/drone_uavgp_ws
source devel/setup.bash
roslaunch mission_master_pkg mm_gazebo_singlewpt.launch
```

## 三、观察执行

程序会自动完成以下流程：

1. 初始化OFFBOARD模式（10秒）
2. 自动解锁并起飞到1.5m
3. 飞往拾取点 (3, 0, 1.5)
4. 飞往第二个拾取点 (3, 3, 1.5)
5. 飞往避障点 (0, 3, 1.5)
6. 飞往避障终点 (-3, 0, 1.5)
7. 飞往追踪点 (-3, -3, 1.5)
8. 返回home点 (0, 0, 1.5)
9. 自动降落

## 四、修改航点

编辑文件：`config/params.yaml`

```yaml
# 修改任意航点坐标（相对home点）
takeoff_pose_z: 1.5    # 起飞高度
pickup_start_pose_x: 3.0  # X坐标
pickup_start_pose_y: 0.0  # Y坐标
# ...根据需要修改其他航点
```

修改后重新启动Mission Master节点即可。

## 五、常用命令

```bash
# 查看当前位置
rostopic echo /iris_0/mavros/local_position/pose

# 查看飞控状态
rostopic echo /iris_0/mavros/state

# 查看所有MAVROS话题
rostopic list | grep mavros
```

## 故障排查

1. **无人机不起飞** → 检查PX4和MAVROS是否都正常运行
2. **节点启动失败** → 检查是否source了工作空间
3. **位置跳动** → Gazebo仿真正常现象，不影响使用
4. **Action相关错误** → 当前版本Action功能未启用，可忽略

完整文档请参考：`README.md`
