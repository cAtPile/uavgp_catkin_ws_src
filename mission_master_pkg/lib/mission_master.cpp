#include "mission_master_pkg/mission_master.h"

/**
 * @brief 构造函数
 */
MissionMaster::MissionMaster() : nh_(""), rate_(20.0)  // 初始化节点句柄和20Hz循环频率
{
    //================初始化订阅者=======================
    state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &MissionMaster::state_cb, this);//状态
    local_pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &MissionMaster::local_pos_cb, this);//位置

    //================初始化发布者======================
    setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/mavros/setpoint_position/local", 10);

    //===============初始化服务客户端=====================
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // 初始化任务状态为等待起飞
    current_mission_state = WAITING_TAKEOFF_STATE;

    // 等待无人机连接
    while (nh_.ok() && !current_state_.connected) {
        ros::spinOnce();
        rate_.sleep();
    }
    
    //加载参数
    loadParam();

    ROS_INFO("MissionMaster initialized. Drone connected.");
}  

/**
 * @brief 析构函数
 */
MissionMaster::~MissionMaster()
{
    ROS_INFO("MissionMaster terminated.");
}

/**
 * @brief 参数加载
 */
void MissionMaster::loadParams(){

    home_pose = current_pose;

    //飞行参数
    nh.param("tolerance_waypoint", TOLERANCE_WAYPOINT, 0.10);

    //航点参数
    nh.param("takeoff_pose_x", TAKEOFF_POSE_X, 0.0);
    nh.param("takeoff_pose_y", TAKEOFF_POSE_Y, 0.0);
    nh.param("takeoff_pose_z", TAKEOFF_POSE_Z, 1.0);
    nh.param("pickup_start_pose_x", PICKUP_START_POSE_X, 1.0);
    nh.param("pickup_start_pose_y", PICKUP_START_POSE_Y, 0.0);
    nh.param("pickup_start_pose_z", PICKUP_START_POSE_Z, 1.0);
    nh.param("pickup_end_pose_x", PICKUP_END_POSE_X, 1.0);
    nh.param("pickup_end_pose_y", PICKUP_END_POSE_Y, 1.0);
    nh.param("pickup_end_pose_z", PICKUP_END_POSE_Z, 1.0);
    nh.param("avoid_start_pose_x", AVOID_START_POSE_X, 0.0);
    nh.param("avoid_start_pose_y", AVOID_START_POSE_Y, 1.0);
    nh.param("avoid_start_pose_z", AVOID_START_POSE_Z, 1.0);
    nh.param("avoid_end_pose_x", AVOID_END_POSE_X, 1.0);
    nh.param("avoid_end_pose_y", AVOID_END_POSE_Y, 0.0);
    nh.param("avoid_end_pose_z", AVOID_END_POSE_Z, 1.0);
    nh.param("trace_start_pose_x", TRACE_START_POSE_X, 1.0);
    nh.param("trace_start_pose_y", TRACE_START_POSE_Y, 1.0);
    nh.param("trace_start_pose_z", TRACE_START_POSE_Z, 1.0);
    nh.param("trace_end_pose_x", TRACE_END_POSE_X, 0.0);
    nh.param("trace_end_pose_y", TRACE_END_POSE_Y, 0.0);
    nh.param("trace_end_pose_z", TRACE_END_POSE_Z, 1.0);

}
