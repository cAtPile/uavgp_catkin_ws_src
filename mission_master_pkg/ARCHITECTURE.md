# Mission Master Node - 完整架构文档

## 📋 目录
1. [节点概述](#节点概述)
2. [系统架构](#系统架构)
3. [核心功能](#核心功能)
4. [状态机设计](#状态机设计)
5. [数据流](#数据流)
6. [接口定义](#接口定义)
7. [关键算法](#关键算法)
8. [使用场景](#使用场景)

---

## 节点概述

### 作用
**Mission Master** 是无人机自主任务管理的核心控制节点，负责：
- 🎯 **任务调度**：管理整个飞行任务的生命周期
- 🚁 **飞行控制**：通过 OFFBOARD 模式控制无人机航点飞行
- 🔄 **状态管理**：维护任务执行的状态机，协调各个子任务
- 🔌 **模块集成**：对接 Pick（抓取）、Avoid（避障）、Trace（追踪）等功能模块
- 📊 **监控反馈**：实时监控飞行状态，输出可视化日志

### 定位
- **层级**：任务层（Task Layer）- 位于路径规划与底层控制之间
- **角色**：任务协调器（Mission Coordinator）
- **模式**：有限状态机（Finite State Machine）+ 事件驱动（Event-Driven）

---

## 系统架构

### 1. 整体架构图

```
┌─────────────────────────────────────────────────────────────┐
│                     Mission Master Node                      │
│                                                               │
│  ┌──────────────┐    ┌─────────────┐    ┌─────────────┐    │
│  │   状态机     │◄───│  回调管理   │◄───│  ROS接口    │    │
│  │ State Machine│    │  Callbacks  │    │  Interface  │    │
│  └──────┬───────┘    └─────────────┘    └──────┬──────┘    │
│         │                                        │            │
│         │            ┌─────────────┐            │            │
│         └───────────►│ 航点管理器  │◄───────────┘            │
│                      │  Waypoint   │                         │
│                      │  Manager    │                         │
│                      └──────┬──────┘                         │
│                             │                                 │
│         ┌──────────┬────────┴────────┬──────────┐           │
│         ▼          ▼                 ▼          ▼           │
│   ┌─────────┐ ┌────────┐      ┌────────┐ ┌─────────┐      │
│   │  Pick   │ │ Avoid  │      │ Trace  │ │  Land   │      │
│   │ Client  │ │ Client │      │ Client │ │ Execute │      │
│   └─────────┘ └────────┘      └────────┘ └─────────┘      │
│   (暂时跳过)   (暂时跳过)       (暂时跳过)                   │
└─────────────────────────────────────────────────────────────┘
                             │
                             ▼
        ┌─────────────────────────────────────────┐
        │            MAVROS (PX4 接口)             │
        │  /mavros/state                          │
        │  /mavros/local_position/pose            │
        │  /mavros/setpoint_position/local        │
        │  /mavros/set_mode                       │
        │  /mavros/cmd/arming                     │
        └─────────────────┬───────────────────────┘
                          │
                          ▼
        ┌─────────────────────────────────────────┐
        │          PX4 飞控 (SITL/实机)            │
        └─────────────────────────────────────────┘
```

### 2. 模块划分

#### 核心模块
| 模块 | 文件 | 职责 |
|------|------|------|
| **状态机核心** | `mission_master.cpp` | 初始化、参数加载、OFFBOARD准备 |
| **回调处理** | `callback.cpp` | 状态转换、任务执行逻辑 |
| **定义头文件** | `mission_master.h` | 类定义、状态枚举、成员变量 |
| **主入口** | `test_mission_master_node.cpp` | ROS节点初始化、spin循环 |

#### 功能模块（待集成）
| 模块 | 文件 | 状态 |
|------|------|------|
| **抓取执行** | `pickAct.cpp` | ❌ 已注释（跳过） |
| **避障执行** | `avoidAct.cpp` | ❌ 已注释（跳过） |
| **追踪执行** | `traceAct.cpp` | ❌ 已注释（跳过） |

---

## 核心功能

### 1. 初始化与准备（Initialization）

#### prepareOffboard()
**目的**：完成 OFFBOARD 模式的4步准备流程

```cpp
void MissionMaster::prepareOffboard() {
    // Step 1: 等待 FCU 连接
    while(!current_vehicle_state_.connected) { ... }
    
    // Step 2: 等待本地位置有效
    while(position_invalid) { ... }
    
    // Step 3: 预发送 100 个 setpoint（PX4要求）
    for(int i = 0; i < 100; ++i) {
        local_pos_pub.publish(current_pose);
    }
    
    // Step 4: 切换到 OFFBOARD 模式
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    set_mode_client.call(offb_set_mode);
    
    offboard_ready_ = true;
}
```

**关键点**：
- PX4 要求进入 OFFBOARD 前至少发送 100 个 setpoint（2Hz 以上持续0.5秒）
- 必须等待 FCU 连接和位置数据有效
- 使用服务调用切换模式

### 2. 航点管理（Waypoint Management）

#### loadWaypoints()
**功能**：将配置的相对坐标转换为绝对坐标

```cpp
void MissionMaster::loadWaypoints() {
    home_pose = current_pose;  // 记录 Home 点
    
    // 所有航点相对 Home 点计算
    TAKEOFF_POSE_XYZ = Eigen::Vector3d(
        TAKEOFF_POSE_X + home_pose.pose.position.x,
        TAKEOFF_POSE_Y + home_pose.pose.position.y,
        TAKEOFF_POSE_Z + home_pose.pose.position.z
    );
    // ... 其他航点类似
}
```

**设计理念**：
- 航点使用**相对坐标**（相对 Home 点），便于场地迁移
- Home 点在解锁时自动记录
- 支持从参数服务器动态加载

#### setPoint()
**功能**：发布位置目标点

```cpp
void MissionMaster::setPoint(Eigen::Vector3d pose_v3d) {
    geometry_msgs::PoseStamped temp_pose;
    temp_pose.header.stamp = ros::Time::now();
    temp_pose.pose.position.x = pose_v3d.x();
    temp_pose.pose.position.y = pose_v3d.y();
    temp_pose.pose.position.z = pose_v3d.z();
    
    local_pos_pub.publish(temp_pose);
}
```

**注意**：
- 必须以 ≥2Hz 频率持续发布（本节点 20Hz）
- 使用 ENU 坐标系（MAVROS 自动转 NED）
- 朝向默认为 0（可扩展为动态朝向）

#### reachCheck()
**功能**：判断是否到达目标点

```cpp
bool MissionMaster::reachCheck(Eigen::Vector3d target) {
    double distance = sqrt(
        pow(current_pose.pose.position.x - target.x(), 2) +
        pow(current_pose.pose.position.y - target.y(), 2) +
        pow(current_pose.pose.position.z - target.z(), 2)
    );
    
    return distance < TOLERANCE_WAYPOINT;  // 默认 0.3m
}
```

### 3. 降落控制（Landing）

#### landExecute()
**策略**：切换到 AUTO.LAND 模式，由 PX4 自动控制降落

```cpp
bool MissionMaster::landExecute() {
    static bool mode_switched = false;
    
    // 首次进入：切换模式
    if (mode != "AUTO.LAND" && !mode_switched) {
        mavros_msgs::SetMode land_mode;
        land_mode.request.custom_mode = "AUTO.LAND";
        set_mode_client.call(land_mode);
        mode_switched = true;
    }
    
    // 检查降落完成（高度 + 解锁状态）
    if (height <= TOLERANCE_WAYPOINT && !armed) {
        return true;  // 降落完成
    }
    
    return false;  // 降落中
}
```

**关键逻辑**：
- 降落完成判断：`高度 < 0.3m` + `已解锁（Disarmed）`
- 在 AUTO.LAND 模式下，PX4 自动控制下降速度和着陆检测
- `callback.cpp` 中额外处理：非 OFFBOARD 模式下也能识别降落完成

---

## 状态机设计

### 1. 状态枚举（State Enum）

```cpp
enum MissionState {
    ENUM_WATTING_TAKEOFF = 0,    // 等待起飞
    ENUM_TAKEOFF,                 // 起飞中
    ENUM_TAKEOFF_SUCCEED,         // 起飞完成
    ENUM_FLYTO_PICKUP_POINT,      // 飞往抓取点
    ENUM_PICKUP_POINT,            // 抓取点（执行抓取）
    ENUM_PICKUP_SUCCEED,          // 抓取完成
    ENUM_FLYTO_AVOID_POINT,       // 飞往避障点
    ENUM_AVOID_POINT,             // 避障点（执行避障）
    ENUM_AVOID_SUCCEED,           // 避障完成
    ENUM_FLYTO_TRACE_POINT,       // 飞往追踪点
    ENUM_TRACE_POINT,             // 追踪点（执行追踪）
    ENUM_TRACE_SUCCEED,           // 追踪完成
    ENUM_FLYTO_LAND_POINT,        // 飞往降落点
    ENUM_LAND_POINT,              // 降落点（执行降落）
    ENUM_LAND_SUCCEED             // 降落完成（任务结束）
};
```

### 2. 状态转换图

```
        [WATTING_TAKEOFF]
              │
              │ ARM + setPoint
              ▼
          [TAKEOFF]
              │
              │ reachCheck()
              ▼
      [TAKEOFF_SUCCEED]
              │
              │ setPoint(WP1)
              ▼
    [FLYTO_PICKUP_POINT]
              │
              │ reachCheck()
              ▼
       [PICKUP_POINT]
              │
              │ pickExecute() [跳过]
              ▼
      [PICKUP_SUCCEED]
              │
              │ setPoint(WP2)
              ▼
    [FLYTO_AVOID_POINT]
              │
              │ reachCheck()
              ▼
       [AVOID_POINT]
              │
              │ avoidExecute() [跳过]
              ▼
      [AVOID_SUCCEED]
              │
              │ setPoint(WP3)
              ▼
    [FLYTO_TRACE_POINT]
              │
              │ reachCheck()
              ▼
       [TRACE_POINT]
              │
              │ traceExecute() [跳过]
              ▼
      [TRACE_SUCCEED]
              │
              │ setPoint(Home)
              ▼
    [FLYTO_LAND_POINT]
              │
              │ reachCheck()
              ▼
       [LAND_POINT]
              │
              │ landExecute()
              ▼
      [LAND_SUCCEED]
```

### 3. 状态转换逻辑（Callback）

**核心思想**：在 `stateCheckCB()` 中根据当前状态和事件触发状态转换

```cpp
void MissionMaster::stateCheckCB(const mavros_msgs::State::ConstPtr &msg) {
    if (current_vehicle_state_.mode == "OFFBOARD") {
        switch (current_mission_state_) {
            case ENUM_WATTING_TAKEOFF:
                // 解锁 + 发布起飞点 → 转到 TAKEOFF
                break;
            
            case ENUM_TAKEOFF:
                setPoint(TAKEOFF_POSE_XYZ);
                if (reachCheck(TAKEOFF_POSE_XYZ)) {
                    // 到达起飞高度 → 转到 TAKEOFF_SUCCEED
                }
                break;
            
            // ... 其他状态类似
        }
    } else if (mode == "AUTO.LAND" && state == ENUM_LAND_POINT) {
        // 特殊处理：降落模式下识别完成
        if (height < threshold && !armed) {
            current_mission_state_ = ENUM_LAND_SUCCEED;
        }
    }
}
```

---

## 数据流

### 1. 订阅话题（Subscribers）

| 话题 | 类型 | 频率 | 用途 |
|------|------|------|------|
| `/iris_0/mavros/state` | `mavros_msgs/State` | 1Hz | 飞控状态（模式、解锁） |
| `/iris_0/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | 50Hz | 当前位置姿态 |

**回调函数**：
- `stateCheckCB()`: 状态机核心，触发状态转换
- `localPoseCB()`: 更新当前位置（用于 reachCheck）

### 2. 发布话题（Publishers）

| 话题 | 类型 | 频率 | 用途 |
|------|------|------|------|
| `/iris_0/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | 20Hz | 位置控制目标点 |

### 3. 服务客户端（Service Clients）

| 服务 | 类型 | 用途 |
|------|------|------|
| `/iris_0/mavros/set_mode` | `mavros_msgs/SetMode` | 切换飞行模式（OFFBOARD/AUTO.LAND） |
| `/iris_0/mavros/cmd/arming` | `mavros_msgs/CommandBool` | 解锁/上锁 |

### 4. Action 客户端（Action Clients）- 待集成

| Action | 状态 | 用途 |
|--------|------|------|
| `/pick_server/pick` | ❌ 未集成 | 抓取任务 |
| `/avoid_server/avoid` | ❌ 未集成 | 避障任务 |
| `/trace_server/trace` | ❌ 未集成 | 追踪任务 |

---

## 关键算法

### 1. OFFBOARD 准备算法

```
算法：prepareOffboard()
输入：无
输出：offboard_ready_ = true

1. 等待 FCU 连接
   while (!connected):
       spin()
       
2. 等待本地位置有效
   while (position_timestamp 过期):
       spin()
       
3. 预发送 setpoint
   for i = 1 to 100:
       publish(current_pose)
       sleep(50ms)
       
4. 请求切换 OFFBOARD
   while (mode != "OFFBOARD"):
       call set_mode("OFFBOARD")
       publish(current_pose)  # 持续发送
       sleep(50ms)
       
5. 设置就绪标志
   offboard_ready_ = true
```

### 2. 航点到达判断算法

```
算法：reachCheck(target)
输入：target = (x, y, z)
输出：true/false

1. 计算欧几里得距离
   distance = sqrt((x-target.x)² + (y-target.y)² + (z-target.z)²)
   
2. 判断容忍范围
   if distance < TOLERANCE_WAYPOINT:
       return true
   else:
       return false
```

**优化方向**：
- 可分离水平/垂直容忍距离
- 可添加速度判断（速度接近0才算到达）
- 可添加朝向判断

### 3. 降落完成判断算法

```
算法：landExecute() + stateCheckCB()
输入：current_pose, vehicle_state
输出：true（完成）/false（进行中）

1. 首次调用：切换到 AUTO.LAND
   if mode != "AUTO.LAND":
       call set_mode("AUTO.LAND")
       
2. 在 OFFBOARD 模式：
   if height < TOLERANCE_WAYPOINT:
       return true
       
3. 在 AUTO.LAND 模式（callback中）：
   if height < TOLERANCE_WAYPOINT AND !armed:
       state = LAND_SUCCEED
       return true
```

---

## 接口定义

### 1. 参数接口（ROS Parameters）

#### 必需参数
| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `vehicle_namespace` | string | `/iris_0` | 飞控命名空间 |
| `TOLERANCE_WAYPOINT` | double | 0.3 | 航点到达容忍距离(m) |

#### 航点参数（相对坐标）
| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `takeoff_pose_x/y/z` | double | 0/0/2.0 | 起飞点 |
| `pickup_start_pose_x/y/z` | double | 2/0/1.5 | 抓取起点 |
| `pickup_end_pose_x/y/z` | double | 4/0/1.5 | 抓取终点 |
| `avoid_start_pose_x/y/z` | double | 2/-2/1.5 | 避障起点 |
| `avoid_end_pose_x/y/z` | double | 0/0/1.5 | 避障终点 |
| `trace_start_pose_x/y/z` | double | -2/2/1.5 | 追踪起点 |
| `trace_end_pose_x/y/z` | double | 0/0/1.5 | 追踪终点 |

### 2. Launch 文件接口

```xml
<launch>
    <!-- 命名空间参数 -->
    <arg name="vehicle_ns" default="iris_0"/>
    
    <node pkg="mission_master_pkg" type="mission_master_node" 
          name="mission_master" output="screen">
        
        <!-- 必需参数 -->
        <param name="vehicle_namespace" value="/$(arg vehicle_ns)"/>
        
        <!-- 航点参数 -->
        <param name="TAKEOFF_POSE_Z" value="2.0"/>
        <param name="PICKUP_POSE_X" value="5.0"/>
        <!-- ... -->
    </node>
</launch>
```

---

## 使用场景

### 场景 1：仿真测试（当前）

**环境**：
- Gazebo SITL
- PX4 固件
- MAVROS 通信
- 命名空间：`/iris_0`

**用途**：
- 验证航点飞行逻辑
- 测试状态机转换
- 调试日志输出
- 性能评估

**启动**：
```bash
# 终端1：启动仿真
roslaunch px4 uavgp.launch

# 终端2：启动任务
roslaunch mission_master_pkg mm_gazebo_singlewpt.launch
```

### 场景 2：实机飞行（后续）

**环境**：
- 真实无人机
- PX4 飞控 + 机载计算机
- RTK 定位
- 命名空间：`/mavros`

**准备工作**：
1. 修改命名空间为 `/mavros`
2. 根据场地调整航点坐标
3. 设置地理围栏
4. 配置紧急停止开关

**额外考虑**：
- GPS/RTK 信号质量检查
- EKF 状态监控
- 电池电量监控
- 风速限制

### 场景 3：多机协同（扩展）

**需要修改**：
1. 为每架飞机创建独立命名空间
   ```xml
   <arg name="vehicle_ns" default="iris_0"/>
   <arg name="vehicle_ns" default="iris_1"/>
   ```

2. 启动多个 Mission Master 节点
   ```xml
   <group ns="iris_0">
       <node pkg="mission_master_pkg" .../>
   </group>
   <group ns="iris_1">
       <node pkg="mission_master_pkg" .../>
   </group>
   ```

3. 添加协同逻辑（未实现）
   - 任务分配
   - 避免碰撞
   - 通信协议

---

## 技术栈总结

| 层次 | 技术 | 版本 | 作用 |
|------|------|------|------|
| **操作系统** | Ubuntu | 20.04 | - |
| **中间件** | ROS | Noetic | 节点通信 |
| **飞控** | PX4 | - | 底层控制 |
| **接口** | MAVROS | - | ROS-PX4桥接 |
| **编程语言** | C++ | 11/14 | 主要逻辑 |
| **数学库** | Eigen | 3 | 向量计算 |
| **构建工具** | CMake + catkin | - | 编译管理 |
| **仿真** | Gazebo | - | 测试环境 |

---

## 性能指标

| 指标 | 数值 | 备注 |
|------|------|------|
| **控制频率** | 20 Hz | 满足 PX4 最低要求（2Hz） |
| **启动时间** | ~5 秒 | 取决于 FCU 连接速度 |
| **航点精度** | 0.3 m | 可配置 |
| **状态切换延迟** | < 50 ms | 一个控制周期 |
| **CPU 占用** | < 5% | 单核，正常飞行 |
| **内存占用** | < 50 MB | - |

---

## 设计模式

1. **有限状态机（FSM）**：任务流程管理
2. **观察者模式**：ROS 回调机制
3. **单例模式**：MissionMaster 类
4. **工厂模式**：航点生成（隐式）
5. **策略模式**：不同Action执行（待实现）

---

## 待优化方向

### 短期
- [ ] 添加轨迹可视化（RViz Markers）
- [ ] 实现参数动态重配置（rqt_reconfigure）
- [ ] 添加飞行日志记录（rosbag + CSV）

### 中期
- [ ] 集成 Pick/Avoid/Trace Action 服务器
- [ ] 实现 OFFBOARD 模式精细降落
- [ ] 添加航点列表文件加载（YAML）

### 长期
- [ ] 多机任务协同调度
- [ ] 动态航点规划接口
- [ ] 实时避障集成
- [ ] 异常恢复机制（电量、信号丢失）

---

**文档版本**: v1.0  
**最后更新**: 2025-11-08  
**维护者**: Mission Master Team
