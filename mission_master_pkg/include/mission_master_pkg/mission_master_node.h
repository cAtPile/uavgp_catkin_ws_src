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
#include <apoc.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <apoc_pkg/detection_data.h>
#include <apoc_pkg/pickup_service.h>
#include <apoc_pkg/avoid_service.h>
#include <apoc_pkg/trace_service.h>
#include <mutex>
#include <vector>

//构建任务枚举
enum mission_state{
    ENUM_WATTING_TAKEOFF,    //等待起飞
    ENUM_TAKEOFF,            //起飞中
    ENUM_TAKEOFF_SUCCEED,    //起飞成功

    ENUM_FLYTO_PICKUP_POINT, //飞往拾取点
    ENUM_PICKUP_POINT,       //执行拾取
    ENUM_PICKUP_SUCCEED,     //拾取成功

    ENUM_FLYTO_AVOID_POINT,  //飞往避障点
    ENUM_AVOID_POINT,        //执行避障
    ENUM_AVOID_SUCCEED,      //避障成功

    ENUM_FLYTO_TRACE_POINT,  //飞往追踪点
    ENUM_TRACE_POINT,        //执行追踪
    ENUM_TRACE_SUCCEED,      //追踪成功

    ENUM_FLYTO_LAND_POINT,   //飞往降落点
    ENUM_LAND_POINT,         //执行降落
    ENUM_LAND_SUCCEED        //降落成功
};

/**
 * @class MissionMaster
 * @brief 任务主控类
 * @details 负责管理无人机任务流程，协调各功能模块执行任务
 */
class MissionMaster{
private:

    //=========数据缓存=============
    //ros句柄
    ros::NodeHandle nh;

    //订阅器
    ros::Subscriber state_sub;          //无人机状态订阅
    ros::Subscriber local_pos_sub;      //本地位置订阅

    //发布器
    ros::Publisher local_pos_pub;       //定点发布

    //服务客户端
    ros::ServiceClient pickup_client;   //拾取服务客户端
    ros::ServiceClient avoid_client;    //避障服务客户端
    ros::ServiceClient trace_client;    //追踪服务客户端

    //任务参数
    double waypoint_tolerance_;         //航点到达容忍距离(m)

    //目标点坐标
    geometry_msgs::PoseStamped pickup_point_;    //拾取点
    geometry_msgs::PoseStamped avoid_point_;     //避障点
    geometry_msgs::PoseStamped trace_point_;     //追踪点
    geometry_msgs::PoseStamped land_point_;      //降落点
    geometry_msgs::PoseStamped home_point_;      // home点

    //当前状态
    mission_state current_mission_state_;  //当前任务状态
    mavros_msgs::State current_vehicle_state_;  //无人机状态
    geometry_msgs::PoseStamped current_pose_;   //当前位置

    //超时计时器
    ros::Time mission_start_time_;       //任务开始时间
    ros::Time state_start_time_;         //当前状态开始时间

    //控制频率
    ros::Rate rate_;                     //循环频率(Hz)
    //--------(数据缓存)------------

    //=========私有函数=============
    /**
     * @brief 加载参数配置
     * @details 从参数服务器加载任务相关参数
     */
    void loadParams();

    /**
     * @brief 无人机状态回调
     * @param msg 无人机状态消息
     */
    void stateCB(const mavros_msgs::State::ConstPtr& msg);

    /**
     * @brief 本地位置回调
     * @param msg 本地位置消息
     */
    void localPoseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);

    /**
     * @brief 解锁检查回调
     * @details 检查无人机是否解锁成功
     */
    void armCheckCB();

    /**
     * @brief 状态检测
     * @param target_state 目标状态
     * @return 是否达到目标状态
     */
    bool stateCheck(mission_state target_state);

    /**
     * @brief 调用拾取服务
     * @return 服务调用是否成功
     */
    bool pickCilent();

    /**
     * @brief 调用避障服务
     * @return 服务调用是否成功
     */
    bool avoidCilent();

    /**
     * @brief 调用跟踪服务
     * @return 服务调用是否成功
     */
    bool traceCilent();

    /**
     * @brief 检查是否到达目标点
     * @param target_pose 目标位置
     * @return 是否到达
     */
    bool isReachTarget(const geometry_msgs::PoseStamped& target_pose);

    //---------(私有函数)-----------

public:
    /**
     * @brief 构造函数
     */
    MissionMaster();

    /**
     * @brief 析构函数
     */
    ~MissionMaster();

    /**
     * @brief 任务状态机运行
     * @details 主循环函数，处理任务状态转换与执行
     */
    void run();

    /**
     * @brief 初始化任务
     * @param home_pose 起始点位置
     * @return 初始化是否成功
     */
    bool initMission(const geometry_msgs::PoseStamped& home_pose);

    /**
     * @brief 获取状态
     */
    void getState();

};

#endif // MISSION_MASTER_NODE_H