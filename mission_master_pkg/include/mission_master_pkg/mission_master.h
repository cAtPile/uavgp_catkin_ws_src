#ifndef MISSION_MASTER_H
#define MISSION_MASTER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <Eigen/Dense>
#include <actionlib/client/simple_action_client.h>
// #include "mission_master_pkg/AvoidAction.h"
// #include "mission_master_pkg/PickAction.h"
// #include "mission_master_pkg/TraceAction.h"
#include "mavros_msgs/SetMode.h"

// 任务状态枚举
enum mission_state
{
    WAITING_TAKEOFF_STATE, // 等待起飞（已初始化，等待OFFBOARD模式）
    EXECUTE_TAKEOFF_STATE, // 执行起飞（解锁并发布起飞点指令）
    SUCCEED_TAKEOFF_STATE, // 起飞成功（悬停在起飞点，等待下一步）

    START_PICKUP_STATE,   // 抓取开始（飞往抓取准备点）
    EXECUTE_PICKUP_STATE, // 执行抓取（降高、闭合爪子、回巡航高度）
    SUCCEED_PICKUP_STATE, // 抓取成功（飞往抓取结束点）

    START_AVOID_STATE,   // 避障开始（飞往避障起始点）
    EXECUTE_AVOID_STATE, // 执行避障（按规划路径避障）
    SUCCEED_AVOID_STATE, // 避障成功（飞往避障结束点）

    START_TRACE_STATE,   // 跟踪开始（飞往跟踪起始点）
    EXECUTE_TRACE_STATE, // 执行跟踪（动态跟随目标）
    SUCCEED_TRACE_STATE, // 跟踪成功（飞往跟踪结束点）

    START_LAND_STATE,   // 降落开始（飞往降落起始点）
    EXECUTE_LAND_STATE, // 执行降落（切换至AUTO.LAND模式）
    SUCCEED_LAND_STATE  // 降落成功（已着陆并上锁）
};

/**
 * @class MissionMaster
 * @brief 任务主控类
 * @details 负责管理无人机任务流程，协调各功能模块执行任务
 */
class MissionMaster
{
private:
    //=========ROS组件=============
    ros::NodeHandle nh_; // 节点句柄

    //---------订阅者---------------
    ros::Subscriber state_sub_;     // 无人机状态订阅者（/mavros/state）
    ros::Subscriber local_pos_sub_; // 局部位置订阅者（/mavros/local_position/pose）
    // ros::Subscriber vision_target_sub_;                   // 视觉目标位姿订阅者
    // ros::Subscriber claw_status_sub_;                     // 机械爪状态订阅者（/claw/status）

    //---------发布者----------------
    ros::Publisher setpoint_pub_; // 位置指令发布者（/mavros/setpoint_position/local）
    // ros::Publisher claw_cmd_pub_;                         // 机械爪控制指令发布者（/claw/cmd）

    //----------客户端---------------
    ros::ServiceClient arming_client_;   // 解锁/上锁服务客户端（/mavros/cmd/arming）
    ros::ServiceClient set_mode_client_; // 模式切换服务客户端（/mavros/set_mode）
    // ros::ServiceClient avoid_plan_client_;                // 避障路径规划服务客户端（/obstacle_avoidance/plan）

    //------------action------------
    // actionlib::SimpleActionClient<mission_master_pkg::AvoidAction> avoid_ac_;  // 避障动作客户端

    //=========飞行参数============
    double pos_tolerance_; // 位置容忍值（到达判定阈值）

    //=========航点参数============
    geometry_msgs::PoseStamped home_pose_;             // 起飞降落点（home位置）
    geometry_msgs::PoseStamped takeoff_waypoint_;      // 起飞点
    geometry_msgs::PoseStamped pickup_start_waypoint_; // 抓取开始点
    geometry_msgs::PoseStamped pickup_end_waypoint_;   // 抓取结束点
    geometry_msgs::PoseStamped avoid_start_waypoint_;  // 避障开始点
    geometry_msgs::PoseStamped avoid_end_waypoint_;    // 避障结束点
    geometry_msgs::PoseStamped trace_start_waypoint_;  // 跟踪开始点
    geometry_msgs::PoseStamped trace_end_waypoint_;    // 跟踪结束点
    geometry_msgs::PoseStamped land_start_waypoint_;   // 降落开始点

    //=========数据缓存============
    mavros_msgs::State current_state_;        // 当前无人机状态
    geometry_msgs::PoseStamped current_pose_; // 当前位置姿态
    mission_state current_mission_state_;     // 当前任务状态

    //=========回调函数============
    /**
     * @brief 无人机状态回调函数
     * @param msg 无人机状态消息
     */
    void state_cb(const mavros_msgs::State::ConstPtr &msg);

    /**
     * @brief 局部位置回调函数
     * @param msg 局部位置姿态消息
     */
    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

    /**
     * @brief 视觉目标位姿回调函数
     * @param msg 视觉目标位置姿态消息
     */
    // void vision_target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

    /**
     * @brief 机械爪状态回调函数
     * @param msg 机械爪状态消息
     */
    // void claw_status_cb(const std_msgs::Bool::ConstPtr& msg);

    //=========私有函数============
    /**
     * @brief 加载参数配置
     * @details 从参数服务器导入全局参数、航点坐标等
     */
    void loadParam();

    /**
     * @brief 等待起飞阶段处理函数
     * @details 发布心跳信号、更新home位置、航点相对化处理
     */
    void waitingTakeoff();

    /**
     * @brief 执行起飞阶段处理函数
     * @details 解锁无人机、发布起飞点指令
     */
    void takeoffExecute();

    /**
     * @brief 发布位置指令
     * @param waypoint 目标航点
     * @details 处理航点相对化，发布到位置控制话题
     */
    void setpointPub(const geometry_msgs::PoseStamped &waypoint);

    /**
     * @brief 执行抓取阶段处理函数
     * @details 降低高度、控制机械爪闭合、返回巡航高度
     */
    void pickupExecute();

    /**
     * @brief 执行避障阶段处理函数
     * @details 调用避障服务，按规划路径执行避障
     */
    void avoidExecute();

    /**
     * @brief 执行跟踪阶段处理函数
     * @details 动态跟随目标，更新跟踪路径
     */
    void traceExecute();

    /**
     * @brief 执行降落阶段处理函数
     * @details 切换至AUTO.LAND模式，完成降落
     */
    void landExecute();

    /**
     * @brief 到达检查函数
     * @param target 目标航点
     * @return 是否到达目标（true-到达，false-未到达）
     * @details 检查当前位置是否在目标位置容忍范围内
     */
    bool reachCheck(const geometry_msgs::PoseStamped &target);

    /**
     * @brief 任务执行状态机
     * @details 根据当前任务状态，调用对应阶段的处理函数
     */
    void missionExecute();

public:
    /**
     * @brief 构造函数
     * @details 初始化ROS组件、变量、参数，设置初始任务状态
     */
    MissionMaster();

    /**
     * @brief 析构函数
     * @details 释放资源
     */
    ~MissionMaster();
};

#endif // MISSION_MASTER_H