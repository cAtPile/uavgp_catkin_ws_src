/**
 * @file mission_master_node.h
 * @brief 无人机任务管理主控节点
 * @details 负责无人机任务流程的状态管理、任务调度与执行监控
 * @date 2025/11/1
 * @par ubuntu 20.04
 *      ros-noetic
 *      px-4,mavros
 */
#ifndef MISSION_MASTER_NODE_H
#define MISSION_MASTER_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <Eigen/Dense>
#include <actionlib/client/simple_action_client.h>
#include "mission_master_pkg/AvoidAction.h"
#include "mission_master_pkg/PickAction.h"
#include "mission_master_pkg/TraceAction.h"
#include "mavros_msgs/SetMode.h"

// 构建任务枚举
enum mission_state
{
    ENMU_TAKEOFF_WAITING, // 等待飞行
    ENMU_TAKEOFF_EXECUTE, // 执行起飞
    ENMU_TAKEOFF_SUCCEED, // 起飞成功

    ENMU_PICK_START,   // 飞到抓取点
    ENMU_PICK_EXECUTE, // 执行抓取
    ENMU_PICK_SUCCEED, // 抓取结束

    ENMU_AVOID_START,   // 飞到避障点
    ENMU_AVOID_EXECUTE, // 执行避障
    ENMU_AVOID_SUCCEED, // 避障成功

    ENMU_TRACE_START,   // 飞到跟踪点
    ENMU_TRACE_EXECUTE, // 执行跟踪
    ENMU_TRACE_SUCCEED, // 跟踪成功

    ENMU_PASS_TP,  // 起飞抓取中间防撞
    ENMU_PASS_PA,  // 抓取避障中间防撞
    ENMU_PASS_AT,  // 避障跟踪中间防撞
    ENMU_PASS_TPA, // 跟踪抓取防撞
    ENMU_PASS_TL,  // 跟踪降落防撞

    ENMU_LAND_START,   // 降落开始
    ENMU_LAND_EXECUTE, // 降落执行
    ENMU_LAND_SUCCEED, // 降落成功
};
/**
 * @class MissionMaster
 * @brief 任务主控类
 * @details 负责管理无人机任务流程，协调各功能模块执行任务
 */
class MissionMaster
{
private:
    //=========全局参数=============
    double TOLERANCE_WAYPOINT; // 航点到达容忍距离(m)
    double TAKEOFF_POSE_X;
    double TAKEOFF_POSE_Y;
    double TAKEOFF_POSE_Z;
    double PICKUP_START_POSE_X;
    double PICKUP_START_POSE_Y;
    double PICKUP_START_POSE_Z;
    double PICKUP_END_POSE_X;
    double PICKUP_END_POSE_Y;
    double PICKUP_END_POSE_Z;
    double AVOID_START_POSE_X;
    double AVOID_START_POSE_Y;
    double AVOID_START_POSE_Z;
    double AVOID_END_POSE_X;
    double AVOID_END_POSE_Y;
    double AVOID_END_POSE_Z;
    double TRACE_START_POSE_X;
    double TRACE_START_POSE_Y;
    double TRACE_START_POSE_Z;
    double TRACE_END_POSE_X;
    double TRACE_END_POSE_Y;
    double TRACE_END_POSE_Z;
    //--------(全局参数)------------

    //=========数据缓存=============
    ros::NodeHandle nh;
    ros::Subscriber state_sub;          // 无人机状态订阅
    ros::Subscriber local_pos_sub;      // 本地位置订阅
    ros::Publisher local_pos_pub;       // 定点发布
    ros::ServiceClient set_mode_client; // 模式设置客户端
    ros::ServiceClient arming_client;

    // 目标点坐标
    Eigen::Vector3d TAKEOFF_POSE_XYZ;
    Eigen::Vector3d PICKUP_START_POSE_XYZ;
    Eigen::Vector3d PICKUP_END_POSE_XYZ;
    Eigen::Vector3d AVOID_START_POSE_XYZ;
    Eigen::Vector3d AVOID_END_POSE_XYZ;
    Eigen::Vector3d TRACE_START_POSE_XYZ;
    Eigen::Vector3d TRACE_END_POSE_XYZ;
    geometry_msgs::PoseStamped home_pose;    // home点
    geometry_msgs::PoseStamped current_pose; // 当前位置

    mission_state current_mission_state_;      // 当前任务状态
    mavros_msgs::State current_vehicle_state_; // 无人机状态

    ros::Time mission_start_time_; // 任务开始时间
    ros::Time state_start_time_;   // 当前状态开始时间
    ros::Rate rate_;               // 循环频率(Hz)
    //--------(数据缓存)------------

    //========action==============
    typedef actionlib::SimpleActionClient<mission_master_pkg::AvoidAction> avoid_Client;
    typedef actionlib::SimpleActionClient<mission_master_pkg::PickAction> pick_Client;
    typedef actionlib::SimpleActionClient<mission_master_pkg::TraceAction> trace_Client;
    avoid_Client avoid_clientor;
    pick_Client pick_clientor;
    trace_Client trace_clientor;
    //-------(action)-------------

    //=========私有函数=============
    void loadParams();                                                 // 参数导入
    void stateCheckCB(const mavros_msgs::State::ConstPtr &msg);        // 无人机状态回调(原stateCheckCB重命名)
    void localPoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg); // 本地位置回调
    bool reachCheck(Eigen::Vector3d pose_v3d);                         // 到达检查
    void loadWaypoints();                                              // 导入航点
    void setPoint(Eigen::Vector3d pose_v3d);                           // 设置目标点

    bool landExecute();

    // Avoid相关回调与执行函数
    void avoidActiveCB();
    void avoidDoneCB(const actionlib::SimpleClientGoalState &state,
                     const mission_master_pkg::AvoidResultConstPtr &result);
    void avoidFeedbackCB(const mission_master_pkg::AvoidFeedbackConstPtr &feedback);
    void avoidAct();
    bool avoidExecute();

    // Pick相关回调与执行函数
    void pickActiveCB();
    void pickDoneCB(const actionlib::SimpleClientGoalState &state,
                    const mission_master_pkg::PickResultConstPtr &result);
    void pickFeedbackCB(const mission_master_pkg::PickFeedbackConstPtr &feedback);
    void pickAct();
    bool pickExecute();

    // Trace相关回调与执行函数
    void traceActiveCB();
    void traceDoneCB(const actionlib::SimpleClientGoalState &state,
                     const mission_master_pkg::TraceResultConstPtr &result);
    void traceFeedbackCB(const mission_master_pkg::TraceFeedbackConstPtr &feedback);
    void traceAct();
    bool traceExecute();
    //---------(私有函数)-----------

public:
    MissionMaster();  // 构造函数
    ~MissionMaster(); // 析构函数
    void getState();  // 获取状态
};

#endif // MISSION_MASTER_NODE_H