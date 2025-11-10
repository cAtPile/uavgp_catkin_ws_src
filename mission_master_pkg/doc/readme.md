# mission_master_pkg 

## 简介

## 文件架构

## 任务流程
1. 初始化
    - ros组件：mavros相关（解锁，状态，位置，模式）；视觉相关；爪子相关；避障相关
    - 变量初始化：全局参数，容忍值，航点，其他
    - 参数导入 loadParam() 

2. 等待起飞 （WAITING_TAKEOFF_STATE）
    - 发布心跳信号
    - 更新home位置
    - 航点相对化
    - 待offb切换到后跟新状态为 EXECUTE_TAKEOFF_SYAYE

3. 开始起飞 （EXECUTE_TAKEOFF_STATE
    - 解锁
    - 发布起飞位置（ TAKEOFF_WAYPOINT ）setpointFly()
    - 更新任务状态为 SUCCEED_TAKEOFF_STATE

4. 起飞成功（ SUCCEED_TAKEOFF_STATE
    - 小循环
    - 发布起飞位置（ TAKEOFF_WAYPOINT ）setpointFly()
    - 到达检查 reachCheck() 成功后切状态为 START_PICKUP_STATE 并退出循环

5. 抓取开始（ START_PICKUP_STATE 
    - 小循环
    - 发布抓取开始点位置（ PICKUP_START_WAYPOINT ）
    - 到达检查 reachCheck() 成功后切状态为 EXECUTE_PICKUP_ STATE 并退出循环

6. 抓取执行 （ EXECUTE_PICKUP_STATE
    - 小循环（抓取函数）
    - 根据视觉消息确定 定点位置 同时降低高度
    - 发布爪子消息
    - 回到巡航高度
    - 更新状态为 SUCCEED_PICKUP_STATE

7. 抓取成功 SUCCEED_PICKUP_STATE
    - 飞到抓取结束点
    - 到达检查
    - 跟新状态为 START_AVOID_STATE

8. 避障 START_AVOID_STATE
    - 飞到避障开始点
    - 到达检查
    - 跟新状态为 EXECUTE_AVOID_STATE

9. 执行避障 EXECUTE_AVOID_SATE
    - 调用避障服务
    - 等待成功后更新状态 SUCCEED_AVOID_STATE

10. 避障成功 SUCCEED_AVOID_STATE
    - 飞到降落结束点
    - 到达检查
    - 更新状态为 START_TRACE_STATE

11. 跟踪开始 START_TRACE_STATE
    - 飞到跟踪开始点
    -
    - 跟新状态为 EXECUTE_TRACE_STATE

12. 跟踪执行 EXECUTE_TRACE_STATE
    - 跟踪函数
    - SUCCEED_TRACE_STATE

13. 跟踪成功SUCCEED_TRACE_STATE
    - 跟踪结束点
    - START_LAND_STATE

14. 降落开始 START_LAND_STATE
    - 降落开始点
    - EXECUTE_LAND_STATE

15. 降落执行 EXECUTE_LAND_STATE
    - 切autoland
    - SUCCEED_LAND_STATE

16. 降落成功 SUCCEED_LAND_STATE
    - 切disarm

## ROS组件列表


## 任务状态列表  
| 状态名称                   | 描述                                  |
|---------------------------|---------------------------------------|
| WAITING_TAKEOFF_STATE     | 等待起飞（已初始化，等待OFFBOARD模式）   |
| EXECUTE_TAKEOFF_STATE     | 执行起飞（解锁并发布起飞点指令）      |
| SUCCEED_TAKEOFF_STATE     | 起飞成功（悬停在起飞点，等待下一步）  |
| START_PICKUP_STATE        | 抓取开始（飞往抓取准备点）            |
| EXECUTE_PICKUP_STATE      | 执行抓取（降高、闭合爪子、回巡航高度）|
| SUCCEED_PICKUP_STATE      | 抓取成功（飞往抓取结束点）            |
| START_AVOID_STATE         | 避障开始（飞往避障起始点）            |
| EXECUTE_AVOID_STATE       | 执行避障（按规划路径避障）            |
| SUCCEED_AVOID_STATE       | 避障成功（飞往避障结束点）            |
| START_TRACE_STATE         | 跟踪开始（飞往跟踪起始点）            |
| EXECUTE_TRACE_STATE       | 执行跟踪（动态跟随目标）              |
| SUCCEED_TRACE_STATE       | 跟踪成功（飞往跟踪结束点）            |
| START_LAND_STATE          | 降落开始（飞往降落起始点）            |
| EXECUTE_LAND_STATE        | 执行降落（切换至AUTO.LAND模式）       |
| SUCCEED_LAND_STATE        | 降落成功（已着陆并上锁）              |

```cpp
enum mission_state
{
    WAITING_TAKEOFF_STATE, // 等待起飞（已初始化，等待OFFBOARD模式）
    EXECUTE_TAKEOFF_STATE, // 执行起飞（解锁并发布起飞点指令）
    SUCCEED_TAKEOFF_STATE, // 起飞成功（悬停在起飞点，等待下一步）

    START_PICKUP_STATE,    // 抓取开始（飞往抓取准备点）
    EXECUTE_PICKUP_STATE,  // 执行抓取（降高、闭合爪子、回巡航高度）
    SUCCEED_PICKUP_STATE,  // 抓取成功（飞往抓取结束点）

    START_AVOID_STATE,     // 避障开始（飞往避障起始点）
    EXECUTE_AVOID_STATE,   // 执行避障（按规划路径避障）
    SUCCEED_AVOID_STATE,   // 避障成功（飞往避障结束点）

    START_TRACE_STATE,     // 跟踪开始（飞往跟踪起始点）
    EXECUTE_TRACE_STATE,   // 执行跟踪（动态跟随目标）
    SUCCEED_TRACE_STATE,   // 跟踪成功（飞往跟踪结束点）
    
    START_LAND_STATE,      // 降落开始（飞往降落起始点）
    EXECUTE_LAND_STATE,    // 执行降落（切换至AUTO.LAND模式）
    SUCCEED_LAND_STATE,    // 降落成功（已着陆并上锁）

    //PASS_A_STATE,          // 防撞途径点预留
}
```

## 航点列表

### 外参数航点
|TAKEOFF_WAYPOINT|起飞点|
|PICKUP_START_WAYPOINT|抓取开始点|
|PICKUP_END_WAYPOINT|抓取结束点|
|AVOID_START_WAYPOINT|避障开始点|
|AVOID_END_WAYPOINT|避障结束点|
|TRACE_START_WAYPOINT|跟踪开始点|
|TRACE_END_WAYPOINT|跟踪结束点|

### 内部航点
|home_pose |起飞降落点|
|current_pose|当前点|



## 参数列表

## 函数功能列表
1. 构造函数
2. 析构函数

3. missionExecute()
    - switch

4. loadParam()
    - 导入外部参数

5. waitingTakeoff()
    - 发布心跳信号
    - 记录home点

6. takeoffExecute()
    - 解锁
    - setpointPub起飞
    - reachCheck
    - 跟新

7. setpointPub()
    - 航点相对化
    - 发布

8. pickupExecute()
    - 略

9. avoidExecute()
    - 略

10. traceExecute()
    - 略

11. landExecute()
    - 切autoland

12. reachCheck()
    - 略

## 全局参数列表