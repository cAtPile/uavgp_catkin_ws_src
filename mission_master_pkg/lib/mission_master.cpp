#include "mission_master_pkg/mission_master.h"

/**
 * @brief 构造函数
 */
MissionMaster::MissionMaster() : nh_(""), rate_(20.0),
                                 gripper_ac_("gripper_action", true)
{
    //================初始化订阅者=======================
    state_sub_ = nh_.subscribe<mavros_msgs::State>("MAV_STATE", 10, &MissionMaster::state_cb, this);                               // 状态
    local_pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("MAV_LOCAL_POSITION_POSE", 10, &MissionMaster::local_pos_cb, this); // 位置
    camtrack_sub_ = nh_.subscribe<mission_master_pkg::CamTrack>("CAM_INFO", 10, &MissionMaster::camtrack_cb, this);                // 视觉

    //================初始化发布者======================
    setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "MAV_SETPOINT_POSITION_LOCAL", 10);

    //===============初始化服务客户端=====================
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("MAV_CMD_ARMING");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("MAV_SET_MODE");

    // 初始化任务状态为等待起飞
    current_mission_state = WAITING_TAKEOFF_STATE;

    // 等待无人机连接
    while (nh_.ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate_.sleep();
    }

    // 加载参数
    loadParams();

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
void MissionMaster::loadParams()
{

    home_pose = current_pose;

    // 话题参数
    nh_.param("MAV_state", MAV_STATE, "/mavros/state");
    nh_.param("MAV_localPositionPose", MAV_LOCAL_POSITION_POSE, "/mavros/local_position/pose");
    nh_.param("MAV_cmdArming", MAV_CMD_ARMING, "/mavros/cmd/arming");
    nh_.param("MAV_setMode", MAV_SET_MODE, "/mavros/set_mode");
    nh_.param("MAV_setpointPositionLocal", MAV_SETPOINT_POSITION_LOCAL, "/mavros/setpoint_position/local");

    nh_.param("CAM_info", CAM_INFO, "/cam_tracker/info");

    // 飞行参数
    nh_.param("tolerance_waypoint", TOLERANCE_WAYPOINT, 0.10);

    // 航点参数
    nh_.param("takeoff_pose_x", TAKEOFF_POSE_X, 0.0);
    nh_.param("takeoff_pose_y", TAKEOFF_POSE_Y, 0.0);
    nh_.param("takeoff_pose_z", TAKEOFF_POSE_Z, 1.0);
    nh_.param("pickup_start_pose_x", PICKUP_START_POSE_X, 1.0);
    nh_.param("pickup_start_pose_y", PICKUP_START_POSE_Y, 0.0);
    nh_.param("pickup_start_pose_z", PICKUP_START_POSE_Z, 1.0);
    nh_.param("pickup_end_pose_x", PICKUP_END_POSE_X, 1.0);
    nh_.param("pickup_end_pose_y", PICKUP_END_POSE_Y, 1.0);
    nh_.param("pickup_end_pose_z", PICKUP_END_POSE_Z, 1.0);
    nh_.param("avoid_start_pose_x", AVOID_START_POSE_X, 0.0);
    nh_.param("avoid_start_pose_y", AVOID_START_POSE_Y, 1.0);
    nh_.param("avoid_start_pose_z", AVOID_START_POSE_Z, 1.0);
    nh_.param("avoid_end_pose_x", AVOID_END_POSE_X, 1.0);
    nh_.param("avoid_end_pose_y", AVOID_END_POSE_Y, 0.0);
    nh_.param("avoid_end_pose_z", AVOID_END_POSE_Z, 1.0);
    nh_.param("trace_start_pose_x", TRACE_START_POSE_X, 1.0);
    nh_.param("trace_start_pose_y", TRACE_START_POSE_Y, 1.0);
    nh_.param("trace_start_pose_z", TRACE_START_POSE_Z, 1.0);
    nh_.param("trace_end_pose_x", TRACE_END_POSE_X, 0.0);
    nh_.param("trace_end_pose_y", TRACE_END_POSE_Y, 0.0);
    nh_.param("trace_end_pose_z", TRACE_END_POSE_Z, 1.0);
}
