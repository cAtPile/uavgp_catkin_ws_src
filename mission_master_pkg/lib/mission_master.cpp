#include "mission_master_pkg/mission_master.h"

/**
 * @brief 构造函数
 */
MissionMaster::MissionMaster() : nh_(""), rate_(20.0),
                                 gripper_ac_("gripper_action", true)
{
    ROS_INFO("MMC");
    // 加载参数
    loadParams();
    //================初始化订阅者=======================
    state_sub_ = nh_.subscribe<mavros_msgs::State>(MAV_STATE, 10, &MissionMaster::state_cb, this); // 状态
    // state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &MissionMaster::state_cb, this);                               // 状态

    local_pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(MAV_LOCAL_POSITION_POSE, 10, &MissionMaster::local_pos_cb, this); // 位置
    // local_pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &MissionMaster::local_pos_cb, this); // 位置

    camtrack_sub_ = nh_.subscribe<mission_master_pkg::CamTrack>(CAM_INFO, 10, &MissionMaster::camtrack_cb, this); // 视觉
    // camtrack_sub_ = nh_.subscribe<mission_master_pkg::CamTrack>("/cam_tracker/info", 10, &MissionMaster::camtrack_cb, this);                // 视觉

    //================初始化发布者======================
    setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(MAV_SETPOINT_POSITION_LOCAL, 10);
    // setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    //===============初始化服务客户端=====================
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(MAV_CMD_ARMING);
    // arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(MAV_SET_MODE);
    // set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // 任务队列
    std::vector<mission_state> mission_queue = {
        WAITING_TAKEOFF_STATE,
        EXECUTE_TAKEOFF_STATE,
        SUCCEED_TAKEOFF_STATE,
        START_LAND_STATE,
        EXECUTE_LAND_STATE,
        SUCCEED_LAND_STATE};

    // 初始化任务状态为等待起飞
    current_mission_state = WAITING_TAKEOFF_STATE;
    /*
        // 等待无人机连接
        while (nh_.ok() && !current_state.connected)
        {
            ros::spinOnce();
            rate_.sleep();
        }
    */
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
 * @brief 参数加载，并打印所有参数值
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

    // 追踪参数
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

    // 航点参数（vector<double>类型）
    nh_.param<std::vector<double>>("takeoff_waypoint_v", takeoff_waypoint_v, {0.0, 0.0, 1.0});
    nh_.param<std::vector<double>>("pickup_start_waypoint_v", pickup_start_waypoint_v, {1.0, 0.0, 1.0});
    nh_.param<std::vector<double>>("pickup_end_waypoint_v", pickup_end_waypoint_v, {1.0, 1.0, 1.0});
    nh_.param<std::vector<double>>("avoid_start_waypoint_v", avoid_start_waypoint_v, {0.0, 1.0, 1.0});
    nh_.param<std::vector<double>>("avoid_end_waypoint_v", avoid_end_waypoint_v, {1.0, 1.0, 1.0});
    nh_.param<std::vector<double>>("trace_start_waypoint_v", trace_start_waypoint_v, {1.0, 0.0, 1.0});
    nh_.param<std::vector<double>>("trace_end_waypoint_v", trace_end_waypoint_v, {0.0, 1.0, 1.0});

    // 任务队列导入
    std::vector<int> default_queue_int = {0, 1, 2, 12, 13, 14, 16}; // 默认任务
    nh_.param<std::vector<int>>("mission_queue", mission_queue_int, default_queue_int);

    mission_queue.clear();
    for (int val : mission_queue_int)
    {
        // 合法性检查：确保int值在枚举有效范围内（0~16）
        if (val >= WAITING_TAKEOFF_STATE && val <= MISSION_SUCCEED_STATE)
        {
            // 强制类型转换：int → mission_state
            mission_queue.push_back(static_cast<mission_state>(val));
        }
        else
        {
            ROS_ERROR("invalid mission_int: %d (0,16) skip ", val);
        }
    }

    // 枚举名称映射表：索引 = 枚举值，元素 = 对应名称（必须与mission_state定义顺序一致）
    const std::vector<std::string> mission_state_names = {
        "WAITING_TAKEOFF_STATE", // 0
        "EXECUTE_TAKEOFF_STATE", // 1
        "SUCCEED_TAKEOFF_STATE", // 2
        "START_PICKUP_STATE",    // 3
        "EXECUTE_PICKUP_STATE",  // 4
        "SUCCEED_PICKUP_STATE",  // 5
        "START_AVOID_STATE",     // 6
        "EXECUTE_AVOID_STATE",   // 7
        "SUCCEED_AVOID_STATE",   // 8
        "START_TRACE_STATE",     // 9
        "EXECUTE_TRACE_STATE",   // 10
        "SUCCEED_TRACE_STATE",   // 11
        "START_LAND_STATE",      // 12
        "EXECUTE_LAND_STATE",    // 13
        "SUCCEED_LAND_STATE",    // 14
        "ERROR_STATE",           // 15
        "MISSION_SUCCEED_STATE"  // 16
    };

    // 验证转换结果
    ROS_INFO("current mission %ld state :", mission_queue.size());
    for (size_t i = 0; i < mission_queue.size(); ++i)
    {
        ROS_INFO("  state %d: %d (%s)",
                 static_cast<int>(i),                                            // 状态索引（0、1、2...）
                 static_cast<int>(mission_queue[i]),                             // 枚举对应的整数
                 mission_state_names[static_cast<int>(mission_queue[i])].c_str() // 枚举名称
        );
    }

    // -------------------------- 打印所有参数（参数展示） --------------------------
    ROS_INFO("\n===== params list =====");

    // 1. 话题参数（字符串类型）
    ROS_INFO("topic list:");
    ROS_INFO("  MAV_state: %s", MAV_STATE.c_str());
    ROS_INFO("  MAV_localPositionPose: %s", MAV_LOCAL_POSITION_POSE.c_str());
    ROS_INFO("  MAV_cmdArming: %s", MAV_CMD_ARMING.c_str());
    ROS_INFO("  MAV_setMode: %s", MAV_SET_MODE.c_str());
    ROS_INFO("  MAV_setpointPositionLocal: %s", MAV_SETPOINT_POSITION_LOCAL.c_str());
    ROS_INFO("  CAM_info: %s", CAM_INFO.c_str());

    // 2. 飞行参数（double类型）
    ROS_INFO("\n fly param:");
    ROS_INFO("  tolerance_waypoint: %.2f (m)", TOLERANCE_WAYPOINT); // 保留2位小数，加单位更清晰

    // 3. 追踪参数（double类型）
    ROS_INFO("\n tracker param:");
    ROS_INFO("  trace_center_x: %.1f (pix)", trace_center_x);
    ROS_INFO("  trace_center_y: %.1f (pix)", trace_center_y);
    ROS_INFO("  cam_loc_rate: %.3f", cam_loc_rate);
    ROS_INFO("  aim_high_trace: %.2f (m)", aim_high_trace);
    ROS_INFO("  tolerace_pix: %.1f (pix)", tolerace_pix);
    ROS_INFO("  step_size_trace: %.2f (m/step)", step_size_trace);
    ROS_INFO("  pickup_center_x: %.1f (pix)", pickup_center_x);
    ROS_INFO("  pickup_center_y: %.1f (pix)", pickup_center_y);
    ROS_INFO("  pickup_aim_high: %.2f (m)", pickup_aim_high);
    ROS_INFO("  pickup_step_size: %.2f (m/step)", pickup_step_size);

    // 4. 航点参数（vector<double>类型，假设为三维坐标 x,y,z）
    ROS_INFO("\n waypoint (x, y, z  (m)):");
    ROS_INFO("  takeoff_waypoint_v: [%.2f, %.2f, %.2f]",
             takeoff_waypoint_v[0], takeoff_waypoint_v[1], takeoff_waypoint_v[2]);
    ROS_INFO("  pickup_start_waypoint_v: [%.2f, %.2f, %.2f]",
             pickup_start_waypoint_v[0], pickup_start_waypoint_v[1], pickup_start_waypoint_v[2]);
    ROS_INFO("  pickup_end_waypoint_v: [%.2f, %.2f, %.2f]",
             pickup_end_waypoint_v[0], pickup_end_waypoint_v[1], pickup_end_waypoint_v[2]);
    ROS_INFO("  avoid_start_waypoint_v: [%.2f, %.2f, %.2f]",
             avoid_start_waypoint_v[0], avoid_start_waypoint_v[1], avoid_start_waypoint_v[2]);
    ROS_INFO("  avoid_end_waypoint_v: [%.2f, %.2f, %.2f]",
             avoid_end_waypoint_v[0], avoid_end_waypoint_v[1], avoid_end_waypoint_v[2]);
    ROS_INFO("  trace_start_waypoint_v: [%.2f, %.2f, %.2f]",
             trace_start_waypoint_v[0], trace_start_waypoint_v[1], trace_start_waypoint_v[2]);
    ROS_INFO("  trace_end_waypoint_v: [%.2f, %.2f, %.2f]",
             trace_end_waypoint_v[0], trace_end_waypoint_v[1], trace_end_waypoint_v[2]);

    ROS_INFO("==========================\n");
}
