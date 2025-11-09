# UAV Autonomous Avoidance & Mission Control System

基于PX4与ROS Noetic的无人机自主避障与任务控制系统，集成激光雷达感知、局部路径规划、任务调度等功能，适用于无人机自主导航场景。


## 项目概述
本项目为无人机提供完整的自主控制解决方案，核心功能包括：
- 基于Mid360激光雷达的三维环境感知与障碍物检测
- 融合直方图与星形搜索的局部避障路径规划
- 无人机任务调度（起飞、跟踪、避障、着陆等）
- PX4飞控通信与控制（通过MAVROS）
- 多传感器定位数据融合（VINS到MAVROS坐标转换）


## 环境依赖
- 操作系统：Ubuntu 20.04
- ROS版本：ROS Noetic
- 飞控系统：PX4
- 通信协议：MAVROS
- 依赖库：
  - C++11及以上
  - Eigen（矩阵运算）
  - PCL（点云处理）
  - actionlib（ROS动作通信）
  - geometry_msgs/mavros_msgs（ROS消息类型）


## 核心功能包

### 1. avoid_planner_pkg（避障规划核心）
- **功能**：基于Mid360激光雷达点云，实现局部路径规划与避障决策
- **核心算法**：
  - 极坐标直方图（Polar Histogram）：将三维点云转换为方位角-俯仰角的距离网格（`dis_map`）
  - 势场法：计算障碍物斥力与目标引力，生成合力方向（`calculateRep`函数）
  - 星形搜索（Star Planner）：在成本矩阵中搜索最优局部路径
- **关键文件**：
  - `local_planner.cpp`：规划器主逻辑
  - `star_planner.cpp`：星形搜索算法实现
  - `pointcloud_processor.cpp`：Mid360点云处理
  - `config/params.yaml`：算法参数（避障距离阈值、搜索深度等）


### 2. apoc_pkg（PX4控制接口）
- **功能**：封装PX4飞控控制接口，实现无人机状态管理与基础动作
- **核心功能**：
  - 无人机解锁/上锁（`armSwitch`）
  - 起飞/着陆/跟踪模式切换
  - 目标点导航与到达检测（`reachCheck`）
  - PID控制（`pidctrl.cpp`）：用于位置/姿态闭环控制
- **关键文件**：
  - `lib/apoc/arm.cpp`：解锁/上锁控制
  - `lib/apoc/avoid.cpp`：避障导航逻辑
  - `src/mission_planner.cpp`：任务规划入口


### 3. mission_master_pkg（任务管理主控）
- **功能**：协调无人机各类任务（避障、拾取、追踪等）的调度与执行
- **核心机制**：基于ROS Action通信，管理多任务间的依赖与优先级
- **关键文件**：
  - `lib/avoidAct.cpp`：避障任务执行逻辑
  - `action/`：定义任务动作消息（Avoid.action、Pick.action等）


### 4. vins_to_mavros（定位数据转换）
- **功能**：将VINS-Mono或激光雷达的定位数据转换为MAVROS兼容格式，用于PX4定位融合
- **关键文件**：
  - `src/otm_node.cpp`：坐标转换与消息发布逻辑


## 目录结构
```
uavgp_catkin_ws_src/
├── avoid_planner_pkg/         # 避障规划包
│   ├── include/               # 头文件（局部规划器、星形搜索等）
│   ├── src/                   # 源文件（路径规划、点云处理等）
│   ├── launch/                # 启动文件（avoidance.launch）
│   └── config/                # 参数配置（params.yaml）
├── apoc_pkg/                  # PX4控制包
│   ├── lib/                   # 核心控制逻辑（解锁、导航、PID等）
│   ├── src/                   # 任务规划实现
│   └── node/                  # 节点入口
├── mission_master_pkg/        # 任务管理包
│   ├── include/               # 任务管理头文件
│   ├── lib/                   # 任务执行逻辑
│   └── action/                # Action消息定义
└── vins_to_mavros/            # 定位转换包
    └── src/                   # 坐标转换实现
```


## 编译与运行

### 编译步骤
1. 克隆仓库到ROS工作空间的`src`目录：
   ```bash
   cd ~/uavgp_catkin_ws/src
   git clone <仓库地址>
   ```

2. 编译工作空间：
   ```bash
   cd ~/uavgp_catkin_ws
   catkin_make
   source devel/setup.bash
   ```


### 启动示例
1. 启动避障系统（含激光雷达处理与路径规划）：
   ```bash
   roslaunch avoid_planner_pkg avoidance.launch
   ```

2. 启动任务控制节点：
   ```bash
   rosrun apoc_pkg main_node
   ```

3. 启动定位转换节点（若使用VINS）：
   ```bash
   rosrun vins_to_mavros otm_node
   ```


## 核心算法说明
1. **避障路径规划流程**：
   - 点云处理：将Mid360激光雷达点云转换为极坐标直方图（`generatePFdismap`）
   - 成本计算：基于障碍物距离计算斥力（`calculateRep`），结合目标方向引力
   - 路径搜索：通过星形搜索树在成本矩阵中寻找最优局部路径

2. **任务调度逻辑**：
   - 基于ROS Action实现任务异步执行与状态反馈
   - 支持任务中断与优先级调整（如避障任务优先于跟踪任务）


## 注意事项
- 确保PX4与MAVROS通信正常（检查`/mavros/state`话题）
- 激光雷达点云需正确转换至机体坐标系（可通过`vins_to_mavros`校准）
- 避障参数（安全距离、斥力系数等）需根据实际场景在`params.yaml`中调整
- 首次运行前建议通过`test/`目录下的单元测试验证核心模块功能


## 维护者
- author：apoc
- 依赖：C++、MAVROS、PX4、ROS Noetic

## 书写规范
- 包类 包名_下划线命名法_pkg
- 节点类 节点名_下划线命名法_node

### 缩写
mm mission_master

### 任务状态类型枚举
  ``` cpp
  ENMU_TAKEOFF_WAITING //等待飞行
  ENMU_TAKEOFF_EXECUTE //执行起飞
  ENMU_TAKEOFF_SUCCEED //起飞成功

  ENMU_PICK_START //飞到抓取点
  ENMU_PICK_EXECUTE //执行抓取
  ENMU_PICK_SUCCEED //抓取结束

  ENMU_AVOID_START //飞到避障点
  ENMU_AVOID_EXECUTE //执行避障
  ENMU_AVOID_SUCCEED //避障成功

  ENMU_TRACE_START //飞到跟踪点
  ENMU_TRACE_EXECUTE //执行跟踪
  ENMU_TRACE_SUCCEED //跟踪成功

  ENMU_PASS_TP //起飞抓取中间防撞
  ENMU_PASS_PA //抓取避障中间防撞
  ENMU_PASS_AT //避障跟踪中间防撞
  ENMU_PASS_TPA //跟踪抓取防撞
  ENMU_PASS_TL //跟踪降落防撞

  ENMU_LAND_START //降落开始
  ENMU_LAND_EXECUTE //降落执行
  ENMU_LAND_SUCCEED //降落成功

  ```

### 视觉action
  ``` action
  # CamTrack.action
  #goal
  bool cam_track_enable #开始反馈消息
  ---
  #result
  bool cam_succeed #任务结束
  ---
  #feedback
  float32 ball_x #球坐标
  float32 ball_y
  float32 car_A_x #车A的坐标
  float32 car_A_y
  float32 car_B_x #B 
  float32 car_B_y  
  float32 car_C_x #C
  float32 car_C_y

  ```

### 爪子action
  ``` action
#Grip.action
# Goal
uint8 GRIP = 0     # 抓取动作
uint8 RELEASE = 1  # 释放动作
uint8 command      # 要执行的命令

---
# Result
bool success       # 是否成功完成
string message     # 执行结果消息

---
# Feedback
int16 current_position  # 当前位置反馈
  ```