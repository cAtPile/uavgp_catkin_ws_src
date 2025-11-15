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
    nh_.param<std::string>("MAV_state", MAV_STATE, "/mavros/state");
    nh_.param<std::string>("MAV_localPositionPose", MAV_LOCAL_POSITION_POSE, "/mavros/local_position/pose");
    nh_.param<std::string>("MAV_cmdArming", MAV_CMD_ARMING, "/mavros/cmd/arming");
    nh_.param<std::string>("MAV_setMode", MAV_SET_MODE, "/mavros/set_mode");
    nh_.param<std::string>("MAV_setpointPositionLocal", MAV_SETPOINT_POSITION_LOCAL, "/mavros/setpoint_position/local");

    nh_.param<std::string>("CAM_info", CAM_INFO, "/cam_tracker/info");

    // 飞行参数
    nh_.param("tolerance_waypoint", TOLERANCE_WAYPOINT, 0.10);

    //追踪参数
    nh_.param("trace_center_x", trace_center_x, 320.0);
    nh_.param("trace_center_y", trace_center_y, 320.0);
    nh_.param("cam_loc_rate", cam_loc_rate, 0.001);
    nh_.param("aim_high_trace", aim_high_trace, 0.1);
    nh_.param("tolerace_pix", tolerace_pix, 10.0);
    nh_.param("step_size_trace", step_size_trace, 0.1);
    nh_.param("pickup_center_x", pickup_center_x, 320.0);
    nh_.param("pickup_center_y", pickup_center_y, 320.0);
    nh_.param("pickup_aim_high", pickup_aim_high, 0.1);
    nh_.param("pickup_step_size", pickup_step_size, 0.1);

    //航点参数
    nh_.param<std::vector<double>>("takeoff_waypoint_v", takeoff_waypoint_v, {0.0,0.0,1.0});
    nh_.param<std::vector<double>>("pickup_start_waypoint_v", pickup_start_waypoint_v, {1.0,0.0,1.0});
    nh_.param<std::vector<double>>("pickup_end_waypoint_v", pickup_end_waypoint_v, {1.0,1.0,1.0});
    nh_.param<std::vector<double>>("avoid_start_waypoint_v", avoid_start_waypoint_v, {0.0,1.0,1.0});
    nh_.param<std::vector<double>>("avoid_end_waypoint_v", avoid_end_waypoint_v, {1.0,1.0,1.0});
    nh_.param<std::vector<double>>("trace_start_waypoint_v", trace_start_waypoint_v, {1.0,0.0,1.0});
    nh_.param<std::vector<double>>("trace_end_waypoint_v", trace_end_waypoint_v, {0.0,1.0,1.0});

}
 