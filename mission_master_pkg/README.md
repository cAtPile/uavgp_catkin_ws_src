# Mission Master Package - 使用说明

## 概述

`mission_master_pkg` 是无人机比赛任务总调度节点，负责协调控制整个任务流程，管理无人机在不同任务区域间的飞行调度。

## 功能特性

- **任务状态机管理**：自动化控制起飞→拾取→避障→追踪→降落全流程
- **OFFBOARD模式控制**：通过MAVROS与PX4飞控通信
- **航点导航**：基于位置反馈的航点到达检测
- **Action接口预留**：支持future集成Pick/Avoid/Trace子任务节点
- **参数化配置**：所有航点通过YAML文件配置，便于调整

## 系统架构

```
mission_master_pkg/
├── action/              # ROS Action定义文件（预留）
│   ├── Avoid.action
│   ├── Pick.action
│   └── Trace.action
├── config/
│   └── params.yaml      # 【重要】所有参数配置文件
├── include/
│   └── mission_master_pkg/
│       └── mission_master.h
├── launch/
│   ├── mm_gazebo_singlewpt.launch  # Gazebo仿真启动文件
│   └── test.launch                  # 简单测试启动文件
├── lib/                 # 核心实现代码
│   ├── mission_master.cpp  # 主类实现
│   ├── callback.cpp        # 状态机回调
│   ├── avoidAct.cpp        # 避障Action（预留）
│   ├── pickAct.cpp         # 拾取Action（预留）
│   └── traceAct.cpp        # 追踪Action（预留）
└── test/
    └── test_mission_master_node.cpp
```

## 任务状态机流程

```
[等待起飞] → [起飞中] → [起飞成功]
    ↓
[飞往拾取点] → [执行拾取(SKIP)] → [拾取完成]
    ↓
[飞往避障点] → [执行避障(SKIP)] → [避障完成]
    ↓
[飞往追踪点] → [执行追踪(SKIP)] → [追踪完成]
    ↓
[飞往降落点] → [执行降落] → [任务完成]
```

**注意**：当前版本Pick/Avoid/Trace任务自动跳过，仅执行点到点飞行。

## 参数配置

所有参数统一在 `config/params.yaml` 中配置：

```yaml
# 无人机命名空间
vehicle_namespace: "/iris_0"

# 飞行参数
tolerance_waypoint: 0.15  # 航点到达容忍距离(米)

# 航点坐标（相对于home点的偏移）
takeoff_pose_x: 0.0
takeoff_pose_y: 0.0
takeoff_pose_z: 1.5   # 起飞高度

pickup_start_pose_x: 3.0
pickup_start_pose_y: 0.0
pickup_start_pose_z: 1.5

pickup_end_pose_x: 3.0
pickup_end_pose_y: 3.0
pickup_end_pose_z: 1.5

avoid_start_pose_x: 0.0
avoid_start_pose_y: 3.0
avoid_start_pose_z: 1.5

avoid_end_pose_x: -3.0
avoid_end_pose_y: 0.0
avoid_end_pose_z: 1.5

trace_start_pose_x: -3.0
trace_start_pose_y: -3.0
trace_start_pose_z: 1.5

trace_end_pose_x: 0.0
trace_end_pose_y: 0.0
trace_end_pose_z: 1.5
```

## 运行步骤

### 1. 编译工作空间

```bash
cd ~/drone_uavgp_ws
catkin build mission_master_pkg
source devel/setup.bash
```

### 2. 启动Gazebo仿真环境

**终端1 - 启动PX4 SITL + Gazebo：**
```bash
cd ~/PX4_Firmware
make px4_sitl gazebo
```

### 3. 启动MAVROS

**终端2 - 启动MAVROS连接：**
```bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

### 4. 启动Mission Master节点

**终端3 - 启动任务节点：**
```bash
cd ~/drone_uavgp_ws
source devel/setup.bash
roslaunch mission_master_pkg mm_gazebo_singlewpt.launch
```

或者使用测试launch文件：
```bash
roslaunch mission_master_pkg test.launch
```

### 5. 监控任务执行

任务会自动按状态机流程执行：

1. **初始化阶段**（约10秒）
   - 等待FCU连接
   - 等待位置数据
   - 预发送setpoint
   - 切换OFFBOARD模式

2. **任务执行阶段**
   - 自动解锁并起飞
   - 依次飞往各个航点
   - 跳过未实现的Action任务
   - 返回home点并降落

3. **完成标志**
   ```
   [OK] Landing completed (Disarmed at X.XX m)
   ========================================
   |    [OK][OK][OK] MISSION COMPLETED! [OK][OK][OK]       |
   ========================================
   ```

## 日志说明

程序使用颜色化日志输出（不含emoji）：

- **CYAN（青色）**：系统初始化信息
- **GREEN（绿色）**：成功状态（起飞/到达航点/完成）
- **BLUE（蓝色）**：航点导航信息
- **YELLOW（黄色）**：警告/跳过任务
- **RED（红色）**：错误信息
- **MAGENTA（品红）**：降落和任务完成

日志格式示例：
```
[OK] Subscribed to: /iris_0/mavros/state
[OK] Publishing to: /iris_0/mavros/setpoint_position/local
[WAIT] Step 1/4: Waiting for FCU connection...
[OK] FCU connected!
[DRONE] MISSION START: TAKEOFF
```

## 调试技巧

### 查看当前位置
```bash
rostopic echo /iris_0/mavros/local_position/pose
```

### 查看飞控状态
```bash
rostopic echo /iris_0/mavros/state
```

### 手动发送位置指令（测试）
```bash
rostopic pub /iris_0/mavros/setpoint_position/local geometry_msgs/PoseStamped \
'{header: {frame_id: "map"}, pose: {position: {x: 0, y: 0, z: 2}}}'
```

### 修改航点参数
直接编辑 `config/params.yaml`，重新启动节点即可。

## 常见问题

### Q1: 节点启动后无人机不起飞？
**A:** 检查：
- MAVROS是否正常连接（`rostopic list | grep mavros`）
- PX4是否在SITL模式运行
- 查看日志是否卡在某个初始化步骤

### Q2: 航点到达检测不准确？
**A:** 调整 `params.yaml` 中的 `tolerance_waypoint` 参数（默认0.15米）

### Q3: 如何修改任务流程？
**A:** 修改 `lib/callback.cpp` 中的 `stateCheckCB()` 函数，按状态机逻辑调整

### Q4: 如何集成Pick/Avoid/Trace子任务？
**A:** 
1. 取消 `mission_master.h` 中Action客户端的注释
2. 在对应的Act.cpp文件中实现Action通信逻辑
3. 修改状态机callback中的跳过逻辑

## 依赖项

- ROS Noetic
- MAVROS
- PX4 SITL
- Gazebo
- Eigen3
- actionlib（预留）

## 技术支持

- 架构文档：`ARCHITECTURE.md`
- 测试文档：`README_TESTING.md`
- Issue反馈：联系开发团队

## 更新日志

**v1.1 (2025-11-08)**
- 移除emoji，优化日志输出
- 统一参数配置到params.yaml
- 清理.bak备份文件
- 完善文档说明

**v1.0**
- 基础任务状态机实现
- OFFBOARD模式控制
- 航点导航功能
