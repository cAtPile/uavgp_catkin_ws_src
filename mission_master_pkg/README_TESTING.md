# Mission Master 测试指南

## ✅ 已完成的优化

### 1. 清晰的启动流程
- Step 1/4: 等待 FCU 连接
- Step 2/4: 等待本地位置
- Step 3/4: 预发送 setpoints（带进度条）
- Step 4/4: 切换到 OFFBOARD 模式

### 2. 优化的日志输出
- ✅ 使用颜色区分不同状态（绿色=成功，黄色=警告，蓝色=任务进度）
- ✅ 图标标记（🚁 ✈ ⊗ 📍 🏠 ⏬ ✓ ✗ ⚡ ⊘）
- ✅ 频繁信息降级为 DEBUG 或限流输出
- ✅ 状态改变时带分隔线，一目了然

### 3. 修复的问题
- ✅ 降落完成后正确识别（AUTO.LAND + Disarmed + 低高度）
- ✅ Action 跳过逻辑（不影响航点飞行）
- ✅ 命名空间支持（`/iris_0`）

---

## 🚀 快速测试

### 终端 1：启动 Gazebo + PX4
```bash
cd ~/PX4_Firmware
source ~/drone_uavgp_ws/devel/setup.bash
roslaunch px4 uavgp.launch
```

### 终端 2：启动 Mission Master（等 Gazebo 完全加载后）
```bash
cd ~/drone_uavgp_ws
source devel/setup.bash
roslaunch mission_master_pkg mm_gazebo_singlewpt.launch
```

---

## �� 预期日志输出

### 启动阶段
```
╔════════════════════════════════════╗
║     INITIALIZING OFFBOARD MODE     ║
╚════════════════════════════════════╝
⏳ Step 1/4: Waiting for FCU connection...
  ✓ FCU connected!
⏳ Step 2/4: Waiting for local position...
  ✓ Position: (0.00, 0.00, 0.00)
⏳ Step 3/4: Pre-sending setpoints...
  Progress: 25%
  Progress: 50%
  Progress: 75%
  Progress: 100%
  ✓ Setpoints ready!
⏳ Step 4/4: Switching to OFFBOARD...
  ✓ OFFBOARD activated!
╔════════════════════════════════════╗
║    ✓ SYSTEM READY FOR MISSION     ║
╚════════════════════════════════════╝
```

### 飞行阶段
```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Vehicle: ✈ ARMED | Mode: OFFBOARD
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
⚡ Arming vehicle...
✓ Vehicle ARMED!
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
🚁 MISSION START: TAKEOFF
   Target: (0.00, 0.00, 2.00)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✓ Takeoff complete!
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
📍 Waypoint 1: (5.00, 0.00, 2.00)
✓ Waypoint 1 reached
⊘ [SKIP] Pickup action
✓ Waypoint 2 reached
✓ Waypoint 3 reached
⊘ [SKIP] Avoid action
✓ Waypoint 4 reached
✓ Waypoint 5 reached
⊘ [SKIP] Trace action
✓ Waypoint 6 reached
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
🏠 Returned to HOME position
⏬ Initiating landing...
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### 降落阶段
```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Vehicle: ✈ ARMED | Mode: AUTO.LAND
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
⏬ Landing... (Height: 1.50 m | Armed)
⏬ Landing... (Height: 0.80 m | Armed)
⏬ Landing... (Height: 0.20 m | Disarmed)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Vehicle: ⊗ DISARMED | Mode: AUTO.LAND
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✓ Landing completed (Disarmed at 0.05 m)
╔════════════════════════════════════╗
║                                    ║
║  ✓✓✓ MISSION COMPLETED! ✓✓✓       ║
║                                    ║
╚════════════════════════════════════╝
```

---

## ⚙️ 参数调整

### 修改航点位置
编辑 `launch/mm_gazebo_singlewpt.launch`：

```xml
<!-- 起飞高度 -->
<param name="TAKEOFF_POSE_Z" value="2.0"/>

<!-- 航点 1（前方5米） -->
<param name="PICKUP_POSE_X" value="5.0"/>
<param name="PICKUP_POSE_Y" value="0.0"/>
<param name="PICKUP_POSE_Z" value="2.0"/>

<!-- 其他航点类似... -->
```

### 修改容忍距离
```xml
<param name="DELTA_POS" value="0.2"/>  <!-- 水平容忍距离 -->
<param name="DELTA_Z" value="0.1"/>    <!-- 垂直容忍距离 -->
```

---

## 🔍 调试技巧

### 1. 查看话题
```bash
# 查看无人机状态
rostopic echo /iris_0/mavros/state

# 查看位置
rostopic echo /iris_0/mavros/local_position/pose

# 查看 setpoint
rostopic echo /iris_0/mavros/setpoint_position/local
```

### 2. 启用 DEBUG 日志
```bash
# 在 launch 文件的 node 标签中添加
<node ... output="screen">
    <env name="ROSCONSOLE_CONFIG_FILE" 
         value="$(find mission_master_pkg)/config/rosconsole.config"/>
</node>
```

然后创建 `config/rosconsole.config`：
```
log4j.logger.ros.mission_master=DEBUG
```

---

## 🛠️ 实机测试准备

### 需要修改的地方

1. **命名空间**：根据实际飞控连接调整
```xml
<arg name="vehicle_ns" default="mavros"/>  <!-- 或你的实际命名空间 -->
```

2. **航点坐标**：根据实际场地调整（相对 Home 点）

3. **安全检查**：
   - 确认 GPS/RTK 信号良好
   - 确认 EKF 状态正常（`/mavros/ekf_status`）
   - 预留足够起飞空间（建议 > 5m x 5m）

4. **紧急处理**：
   - 随时准备切换到手动模式（RC 开关）
   - 监控电池电量
   - 设置地理围栏

---

## 📝 已知限制

1. **Action 服务器未集成**：当前 Pick/Avoid/Trace 任务被跳过
2. **降落模式**：使用 AUTO.LAND，无法精确控制降落速度
3. **无避障**：纯航点飞行，无动态避障功能

---

## 📧 问题反馈

如有问题，请检查：
1. 日志输出是否有错误/警告
2. 话题是否正常发布（`rostopic list | grep iris_0`）
3. Gazebo 中无人机是否正常显示
