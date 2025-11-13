
/*
                       _oo0oo_
                      o8888888o
                      88" . "88
                      (| -_- |)
                      0\  =  /0
                    ___/`---'\___
                  .' \\|     |# '.
                 / \\|||  :  |||# \
                / _||||| -:- |||||- \
               |   | \\\  -  #/ |   |
               | \_|  ''\---/''  |_/ |
               \  .-\__  '-'  ___/-. /
             ___'. .'  /--.--\  `. .'___
          ."" '<  `.___\_<|>_/___.' >' "".
         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
         \  \ `_.   \_ __\ /__ _/   .-` /  /
     =====`-.____`.___ \_____/___.-`___.-'=====
                       `=---='


     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

               佛祖保佑         永无BUG

*/

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
#include "mavros_msgs/SetMode.h"
#include <actionlib/client/simple_action_client.h>
#include "mission_master_pkg/GripAction.h"
#include <mission_master_pkg/CamTrack.h>

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
    SUCCEED_LAND_STATE, // 降落成功（已着陆并上锁）

    ERROR_STATE,          // 解锁异常状态
    MISSION_SUCCEED_STATE // 任务完全成功
    // 预留状态
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
    ros::Rate rate_;     // 循环频率(Hz)

    //---------订阅者---------------
    ros::Subscriber state_sub_;     // 无人机状态订阅者（/mavros/state）
    ros::Subscriber local_pos_sub_; // 局部位置订阅者（/mavros/local_position/pose）
    ros::Subscriber camtrack_sub_;  // 视觉订阅
    // ros::Subscriber 视觉，爪子 预留

    //---------发布者----------------
    ros::Publisher setpoint_pub_; // 位置指令发布者（/mavros/setpoint_position/local）
    // ros::Publisher 视觉，爪子 预留

    //----------客户端---------------
    ros::ServiceClient arming_client_;   // 解锁/上锁服务客户端（/mavros/cmd/arming）
    ros::ServiceClient set_mode_client_; // 模式切换服务客户端（/mavros/set_mode）

    //------------action------------
    // actionlib::SimpleActionClient 避障预留
    actionlib::SimpleActionClient<mission_master_pkg::GripAction> gripper_ac_; // 爪子Action客户端

    //=========飞行参数============
    double TOLERANCE_WAYPOINT; // 位置容忍值（到达判定阈值）

    //=========航点参数============
    geometry_msgs::PoseStamped home_pose; // 起飞降落点（home位置）

    Eigen::Vector3d TAKEOFF_WAYPOINT;
    Eigen::Vector3d PICKUP_START_WAYPOINT;
    Eigen::Vector3d PICKUP_END_WAYPOINT;
    Eigen::Vector3d AVOID_START_WAYPOINT;
    Eigen::Vector3d AVOID_END_WAYPOINT;
    Eigen::Vector3d TRACE_START_WAYPOINT;
    Eigen::Vector3d TRACE_END_WAYPOINT;

    //--------航点缓存XYZ----------
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

    //=========数据缓存============
    mavros_msgs::State current_state;              // 当前无人机状态
    geometry_msgs::PoseStamped current_pose;       // 当前位置姿态
    geometry_msgs::PoseStamped temp_pose;          // 临时位置姿态
    mission_state current_mission_state;           // 当前任务状态
    mission_master_pkg::CamTrack current_camtrack; // 当前视觉

    //=========回调函数============
    void state_cb(const mavros_msgs::State::ConstPtr &msg) { current_state = *msg; }                 // 无人机状态回调
    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) { current_pose = *msg; }      // 本地位置回调
    void camtrack_cb(const mission_master_pkg::CamTrack::ConstPtr &msg) { current_camtrack = *msg; } // 视觉回调
    void gripperFeedbackCb(const mission_master_pkg::GripFeedback::ConstPtr &feedback) {}

    //=========私有函数============
    void loadParams();                            // 参数导入
    void loadWaypoints();                         // 航点导入
    void waitingTakeoff();                        // 等待起飞
    bool armSet();                                // 解锁
    void takeoffExecute();                        // 执行起飞
    void takeoffCheck();                          // 起飞成功检查
    void setPoint(Eigen::Vector3d set_point);     // 航点飞行
    bool reachCheck(Eigen::Vector3d check_point); // 到达检查

    void pickupStart();   // 开始拾取
    void pickupExecute(); // 抓取执行
    void pickupCheck();   // 拾取成功检查
    void pickLoop();      // 抓循环
    bool gripPick();      // 爪子抓

    void avoidStart();   // 避障开始
    void avoidExecute(); // 避障执行
    void avoidCheck();   // 避障检查

    void traceStart();   // 跟踪开始
    void traceExecute(); // 追踪执行
    void traceCheck();   // 跟踪检查
    void traceLoop();    //
    void gripRelease();  //

    void landStart();   // 降落开始
    void landExecute(); // 降落执行
    void landCheck();   // 降落检查

    void missionExecute(); // 任务执行（主循环）

public:
    MissionMaster();  // 构造函数
    ~MissionMaster(); // 析构函数
    void run();       // 主任务
};

#endif // MISSION_MASTER_H