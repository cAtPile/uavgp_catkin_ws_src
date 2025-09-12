# px4 ros-noetic autopiolot control

***

author：apoc
email：
ros:ros-noetic
ubuntu:20.04
hardwear:px4
depend:cpp,mavros

***

## 文件结构

apoc_pkg
- config
-- params.xml           参数
- include
-- apoc.h               头文件
- launch
-- apoc_test.launch     启动
- lib
-- apoc.cpp             构造函数
-- connect.cpp          连接
-- mode.cpp             模式切换
-- arm.cpp              解锁
-- reach.cpp            到达检查
-- fly_ab.cpp           绝对飞行
-- fly_re.cpp           相对修正
-- pid_flyab.cpp        pid修正
-- track.cpp            追踪
-- takeoff.cpp          起飞
-- land.cpp             降落
-- hover.cpp            悬停
- msg                   消息
- src                   源代码
-- test_node.cpp        测试

## 函数列表

---

class apoc;                         apoc类
apoc::apoc();                       构造函数  
bool connectSwitch();               连接
bool modeSwitch(int mode_key);      模式切换
bool armSwitch(int arm_key);        解锁
bool reachCheck(float check_x , float check_y , float check_z , float check_yaw );                  到达检查
bool flytoAbsolute(float fly_ab_x , float fly_ab_y , float fly_ab_z , float fly_ab_yaw );           绝对飞行
bool flytoPIDcorrect(float fly_pid_x , float fly_pid_y , float fly_pid_z , float fly_pid_yaw );     pid修正
bool flytoRelative(float fly_re_x , float fly_re_y , float fly_re_z , float fly_re_yaw );           相对修正
待定track.cpp                           追踪
bool takeoffSwitch(float takeoff_alt)   起飞
bool landSwitch()                       降落
bool hoverSwitch(float hover_time)      悬停    



---

1. ### connectSwitch

/*
* file:     conect.cpp
* path:     /lib
* name:     connectSwitch
* describe: 确保完成连接
* input:    NONE
* output:   true    ->  连接成功
*           false   ->  连接失败
* param:    CONNECT_TIMEOUT
* value:    current_pose
* depend:   ros-noetic,cpp,mavros,px4,ununtu20.04,apoc.h
* function: NONE
* vision:   1.0
* method:   循环等待连接,连接成功持续发布current_pose,防止断开
* info:     "ROS node is not running properly during connect"
*           连接时ros不正常运行
*           
*           "Waiting for FCU connection..."
*           等待连接
*
*           "Failed to connect to FCU , timeout"
*           超时连接失败
*
*           "FCU connection failed unkunown reason unexpectedly!"
*           未知原因失败
*
*/

2. ### modeSwitch

/*
* file：    mode.cpp
* name：    modeSwitch
* path:     /lib
* describe：switch mode
* input：   mode_key = 1        ->  offboard
*           mode_key = 0/其他   ->  自稳
* output:   true    ->  成功切换到目标模式
*           false   ->  未切换到目标模式
* param:    MODESWITCH_TIIMEOUT
* value:    
* depend:   ros-noetic,cpp,mavros,px4,ununtu20.04,apoc.h
* function: NONE
* vision:   1.0
* method:   
* info:     "Not connected to FCU, cannot set mode"
*           未连接无法设置
*
*           "Timeout setting " << target_mode << " mode"
*           setmode超时
*
*           "ROS node is not running properly during setmode"
*           setmode时ros节点失效
*
*           "Modeset failed unkunown reason unexpectedly!"
*           未知原因失效
*/

3. ### armSwitch
/*
* file:     arm.cpp
* name：    armSwitch
* path：    /lib
* describe：arm/disarm
* input：   arm_key = 0 -> disarm
*           arm_key = 1 -> arm
* output:   true    ->  成功arm/disarm
*           false   ->  失败
* param:    ARMSWITCH_TIMEOUT
* depend:   ros-noetic,cpp,mavros,px4,ununtu20.04,apoc.h
* function: NONE
* vision:   1.0
* method:   调用arm_cmd
* info:     "CANNOT arm/disarm ! DISCONNECT to FCU"
*           因未连接而解锁/上锁失败
*           
*           "Cannot arm/disarm ! not in OFFBOARD mode"
*           因未offboard而解锁/上锁失败
*
*           "Arm/disarm TIMEOUT during"
*           解锁/上锁超时
*
*           "Arm/disarm failed UNKUNOWN reason unexpectedly!"
*           未知原因失败
*   
*           "ROS node is not running ,during arm/disarm"
*           ROS非正常
*/

4. ### reachCheck

/*
* name：    reachCheck
* describe：检查是否到达目标check_x/y/z/yaw
* input：   check_x/y/z/yaw
* output:   true    ->  到达
*           false   ->  未到达
* param:    REACH_TIMEOUT,REACH_TOLERANCE_DISTANCE,REACH_TOLERANCE_ANGLE
* depend:   ros-noetic,cpp,mavros,px4,ununtu20.04,apoc.h
* function: NONE
* vision:   1.0
*/

5. ### flytoAbsolute

/*
* name：    flytoAbsolute
* describe：飞行到绝对坐标fly_ab_x/y/z/yaw
* input：   fly_ab_x/y/z/yaw
* output:   true    ->  到达
*           false   ->  未到达
* param:    NONE
* depend:   ros-noetic,cpp,mavros,px4,ununtu20.04,apoc.h
* function: reachCheck
* vision:   1.0
* method：  定点发布坐标，然后reachCheck检查到达
*/

6. ### flytoPIDcorrect

/*
* name：    flytoPIDcorrect
* describe：使用PID控制飞行到绝对坐标fly_ab_x/y/z/yaw
* input：   fly_pid_x/y/z/yaw
* output:   true    ->  到达
*           false   ->  未到达
* param:    NONE
* depend:   ros-noetic,cpp,mavros,px4,ununtu20.04,apoc.h
* function: reachCheck，flytoAbsolute
* vision:   1.0
* method：  使用pid控制器计算步长，使用flytoAbsolute飞到中间步，直到到达目的地，reachCheck检查到达
*/

7. ### flytoRelative

/*
* name：    flytoRelative
* describe：飞行到相对home位姿的坐标fly_ab_x/y/z/yaw
* input：   fly_re_x/y/z/yaw
* output:   true    ->  到达
*           false   ->  未到达
* param:    NONE
* depend:   ros-noetic,cpp,mavros,px4,ununtu20.04,apoc.h
* function: reachCheck，flytoAbsolute/flytoPIDcorrect
* vision:   1.0
* method：  fly_ab_x/y/z/yaw+home的位置姿态，使用flytoAbsolute/flytoPIDcorrect到目的地
*/

8. ### 待定track.cpp

9. ### takeoffSwitch

/*
* name：    takeoffSwitch
* describe：起飞，并记录home位置
* input：   takeoff_alt
* output:   true    ->  起飞
*           false   ->  无法起飞
* param:    NONE
* depend:   ros-noetic,cpp,mavros,px4,ununtu20.04,apoc.h
* function: reachCheck，flytoAbsolute/flytoPIDcorrect
* vision:   1.0
* method：  记录起飞前位置，加上起飞高度，使用flytoAbsolute/flytoPIDcorrect到目的地
*/

10. ### landSwitch

/*
* name：    landSwitch
* describe：降落
* input：   NONE
* output:   true    ->  降落成功
*           false   ->  降落失败
* param:    LANDING_TIMEOUT,LANDING_TOLERANCE
* depend:   ros-noetic,cpp,mavros,px4,ununtu20.04,apoc.h
* function: reachCheck，flytoAbsolute/flytoPIDcorrect
* vision:   1.0
* method：  记录当前位置，使用flytoAbsolute/flytoPIDcorrect到当前x/y/LANDING_TOLERANCE/yaw,然后disarm
*/

11. ### hoverSwitch

/*
* name：    hoverSwitch
* describe：悬停
* input：   hover_time
* output:   true    ->  悬停成功
*           false   ->  悬停失败
* param:    NONE
* depend:   ros-noetic,cpp,mavros,px4,ununtu20.04,apoc.h
* function: NONE
* vision:   1.0
* method：  维持发布当前位置
*/

### 工具函数

### 暴露函数

## 参数列表

CONNECT_TIMEOUT             连接超时
MODESWITCH_TIIMEOUT         切换模式超时
ARMSWITCH_TIMEOUT           arm/disarm超时
REACH_TIMEOUT               到达检查超时
REACH_TOLERANCE_DISTANCE    到达检查距离容忍值
REACH_TOLERANCE_ANGLE       到达检查角度容忍值
LANDING_TIMEOUT             降落超时
LANDING_TOLERANCE           降落容忍值
FLY_AB_TIMEOUT              绝对飞行超时
