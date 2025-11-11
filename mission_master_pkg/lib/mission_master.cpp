#include "mission_master.h"

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

    //加载参数
    loadParam();

    // 初始化任务状态为等待起飞
    current_mission_state_ = WAITING_TAKEOFF_STATE;

    // 等待无人机连接
    while (nh_.ok() && !current_state_.connected) {
        ros::spinOnce();
        rate_.sleep();
    }
    ROS_INFO("MissionMaster initialized. Drone connected.");
}  

/**
 * @brief 析构函数
 */
MissionMaster::~MissionMaster()
{
    ROS_INFO("MissionMaster terminated.");
}