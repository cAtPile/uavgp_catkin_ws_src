/**
 * @file mission_master_node.cpp
 * @brief 任务主控节点实现
 * @date 2025/11/1
 * @note 优化日志输出，添加颜色标记
 */
#include "mission_master_pkg/mission_master.h"
#include <cmath>
#include <ros/console.h>

// ANSI 颜色代码
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

MissionMaster::MissionMaster() : rate_(20.0), 
                                 current_mission_state_(ENUM_WATTING_TAKEOFF),
                                 offboard_ready_(false)
{
    // 获取命名空间参数（默认为 /iris_0）
    nh.param<std::string>("vehicle_namespace", vehicle_namespace_, "/iris_0");
    ROS_INFO(ANSI_COLOR_CYAN "========================================" ANSI_COLOR_RESET);
    ROS_INFO(ANSI_COLOR_CYAN "   Mission Master Node Starting..." ANSI_COLOR_RESET);
    ROS_INFO(ANSI_COLOR_CYAN "========================================" ANSI_COLOR_RESET);
    ROS_INFO(ANSI_COLOR_BLUE "Vehicle namespace: %s" ANSI_COLOR_RESET, vehicle_namespace_.c_str());

    // 初始化订阅器（添加命名空间前缀）
    std::string state_topic = vehicle_namespace_ + "/mavros/state";
    std::string local_pos_topic = vehicle_namespace_ + "/mavros/local_position/pose";
    
    state_sub = nh.subscribe<mavros_msgs::State>(state_topic, 10, &MissionMaster::stateCheckCB, this);
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(local_pos_topic, 10, &MissionMaster::localPoseCB, this);
    
    ROS_INFO(ANSI_COLOR_GREEN "[OK] Subscribed to: %s" ANSI_COLOR_RESET, state_topic.c_str());
    ROS_INFO(ANSI_COLOR_GREEN "[OK] Subscribed to: %s" ANSI_COLOR_RESET, local_pos_topic.c_str());

    // 初始化发布器（添加命名空间前缀）
    std::string setpoint_topic = vehicle_namespace_ + "/mavros/setpoint_position/local";
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(setpoint_topic, 10);
    ROS_INFO(ANSI_COLOR_GREEN "[OK] Publishing to: %s" ANSI_COLOR_RESET, setpoint_topic.c_str());

    // 加载参数配置
    loadParams();

    // 初始化模式设置客户端（添加命名空间前缀）
    std::string set_mode_srv = vehicle_namespace_ + "/mavros/set_mode";
    std::string arming_srv = vehicle_namespace_ + "/mavros/cmd/arming";
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(set_mode_srv);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>(arming_srv);

    current_pose.header.frame_id = "map";
    current_pose.pose.position.x = 0.0;
    current_pose.pose.position.y = 0.0;
    current_pose.pose.position.z = 0.0;
    current_pose.pose.orientation.x = 0.0;
    current_pose.pose.orientation.y = 0.0;
    current_pose.pose.orientation.z = 0.0;
    current_pose.pose.orientation.w = 1.0;
    current_pose.header.stamp = ros::Time::now();

    // 准备 OFFBOARD 模式
    prepareOffboard();
}

MissionMaster::~MissionMaster()
{
    ROS_INFO(ANSI_COLOR_YELLOW "Mission Master destroyed" ANSI_COLOR_RESET);
}

void MissionMaster::prepareOffboard()
{
    ROS_INFO(ANSI_COLOR_CYAN "=════════════════════════════════════╗" ANSI_COLOR_RESET);
    ROS_INFO(ANSI_COLOR_CYAN "|     INITIALIZING OFFBOARD MODE     |" ANSI_COLOR_RESET);
    ROS_INFO(ANSI_COLOR_CYAN "=════════════════════════════════════╝" ANSI_COLOR_RESET);
    
    // 等待 FCU 连接
    ROS_INFO("[WAIT] Step 1/4: Waiting for FCU connection...");
    int wait_count = 0;
    while(ros::ok() && !current_vehicle_state_.connected) {
        ros::spinOnce();
        rate_.sleep();
        wait_count++;
        if (wait_count % 40 == 0) {  // 每2秒提示一次（20Hz）
            ROS_WARN("  Still waiting... (%d seconds)", wait_count / 20);
        }
    }
    ROS_INFO(ANSI_COLOR_GREEN "  [OK] FCU connected!" ANSI_COLOR_RESET);

    // 等待获取有效的本地位置
    ROS_INFO("[WAIT] Step 2/4: Waiting for local position...");
    ros::Time last_request = ros::Time::now();
    while(ros::ok() && (ros::Time::now() - current_pose.header.stamp) > ros::Duration(1.0)) {
        ros::spinOnce();
        rate_.sleep();
        if(ros::Time::now() - last_request > ros::Duration(3.0)) {
            ROS_WARN("  Still waiting for position data...");
            last_request = ros::Time::now();
        }
    }
    ROS_INFO(ANSI_COLOR_GREEN "  [OK] Position: (%.2f, %.2f, %.2f)" ANSI_COLOR_RESET, 
             current_pose.pose.position.x,
             current_pose.pose.position.y,
             current_pose.pose.position.z);

    // 预先发送 setpoint
    ROS_INFO("[WAIT] Step 3/4: Pre-sending setpoints...");
    int progress = 0;
    for(int i = 0; i < 100; ++i) {
        local_pos_pub.publish(current_pose);
        ros::spinOnce();
        rate_.sleep();
        
        int new_progress = (i + 1) * 100 / 100;
        if (new_progress > progress && new_progress % 25 == 0) {
            ROS_INFO("  Progress: %d%%", new_progress);
            progress = new_progress;
        }
    }
    ROS_INFO(ANSI_COLOR_GREEN "  [OK] Setpoints ready!" ANSI_COLOR_RESET);

    // 切换到 OFFBOARD 模式
    ROS_INFO("[WAIT] Step 4/4: Switching to OFFBOARD...");
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    
    ros::Time last_switch_request = ros::Time::now();
    while(ros::ok() && current_vehicle_state_.mode != "OFFBOARD") {
        if(ros::Time::now() - last_switch_request > ros::Duration(2.0)) {
            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("  Requesting mode switch...");
            } else {
                ROS_WARN("  Retrying mode switch...");
            }
            last_switch_request = ros::Time::now();
        }
        local_pos_pub.publish(current_pose);
        ros::spinOnce();
        rate_.sleep();
    }
    ROS_INFO(ANSI_COLOR_GREEN "  [OK] OFFBOARD activated!" ANSI_COLOR_RESET);

    offboard_ready_ = true;
    
    ROS_INFO(ANSI_COLOR_GREEN "=════════════════════════════════════╗" ANSI_COLOR_RESET);
    ROS_INFO(ANSI_COLOR_GREEN "|    [OK] SYSTEM READY FOR MISSION     |" ANSI_COLOR_RESET);
    ROS_INFO(ANSI_COLOR_GREEN "=════════════════════════════════════╝" ANSI_COLOR_RESET);
}

bool MissionMaster::reachCheck(Eigen::Vector3d target)
{
    double distance = std::sqrt(
        std::pow(current_pose.pose.position.x - target.x(), 2) +
        std::pow(current_pose.pose.position.y - target.y(), 2) +
        std::pow(current_pose.pose.position.z - target.z(), 2)
    );
    
    // 降级为 DEBUG，避免刷屏
    ROS_DEBUG("Distance to target: %.2f m (tolerance: %.2f m)", distance, TOLERANCE_WAYPOINT);
    
    return distance < TOLERANCE_WAYPOINT;
}

void MissionMaster::loadParams()
{
    nh.param("TOLERANCE_WAYPOINT", TOLERANCE_WAYPOINT, 0.3);
    nh.param("takeoff_pose_x", TAKEOFF_POSE_Z, 0.0);
    nh.param("takeoff_pose_y", TAKEOFF_POSE_Z, 0.0);
    nh.param("takeoff_pose_z", TAKEOFF_POSE_Z, 2.0);
    nh.param("pickup_start_pose_x", PICKUP_START_POSE_X, 2.0);
    nh.param("pickup_start_pose_y", PICKUP_START_POSE_Y, 0.0);
    nh.param("pickup_start_pose_z", PICKUP_START_POSE_Z, 1.5);
    nh.param("pickup_end_pose_x", PICKUP_END_POSE_X, 4.0);
    nh.param("pickup_end_pose_y", PICKUP_END_POSE_Y, 0.0);
    nh.param("pickup_end_pose_z", PICKUP_END_POSE_Z, 1.5);
    nh.param("avoid_start_pose_x", AVOID_START_POSE_X, 2.0);
    nh.param("avoid_start_pose_y", AVOID_START_POSE_Y, -2.0);
    nh.param("avoid_start_pose_z", AVOID_START_POSE_Z, 1.5);
    nh.param("avoid_end_pose_x", AVOID_END_POSE_X, 0.0);
    nh.param("avoid_end_pose_y", AVOID_END_POSE_Y, 0.0);
    nh.param("avoid_end_pose_z", AVOID_END_POSE_Z, 1.5);
    nh.param("trace_start_pose_x", TRACE_START_POSE_X, -2.0);
    nh.param("trace_start_pose_y", TRACE_START_POSE_Y, 2.0);
    nh.param("trace_start_pose_z", TRACE_START_POSE_Z, 1.5);
    nh.param("trace_end_pose_x", TRACE_END_POSE_X, 0.0);
    nh.param("trace_end_pose_y", TRACE_END_POSE_Y, 0.0);
    nh.param("trace_end_pose_z", TRACE_END_POSE_Z, 1.5);
    
    ROS_INFO(ANSI_COLOR_BLUE "[OK] Parameters loaded (Waypoint tolerance: %.2f m)" ANSI_COLOR_RESET, 
             TOLERANCE_WAYPOINT);
}

void MissionMaster::loadWaypoints()
{
    home_pose = current_pose;
    TAKEOFF_POSE_XYZ = Eigen::Vector3d(TAKEOFF_POSE_X + home_pose.pose.position.x,
                                       TAKEOFF_POSE_Y + home_pose.pose.position.y,
                                       TAKEOFF_POSE_Z + home_pose.pose.position.z);
    PICKUP_START_POSE_XYZ = Eigen::Vector3d(PICKUP_START_POSE_X + home_pose.pose.position.x,
                                            PICKUP_START_POSE_Y + home_pose.pose.position.y,
                                            PICKUP_START_POSE_Z + home_pose.pose.position.z);
    PICKUP_END_POSE_XYZ = Eigen::Vector3d(PICKUP_END_POSE_X + home_pose.pose.position.x,
                                          PICKUP_END_POSE_Y + home_pose.pose.position.y,
                                          PICKUP_END_POSE_Z + home_pose.pose.position.z);
    AVOID_START_POSE_XYZ = Eigen::Vector3d(AVOID_START_POSE_X + home_pose.pose.position.x,
                                           AVOID_START_POSE_Y + home_pose.pose.position.y,
                                           AVOID_START_POSE_Z + home_pose.pose.position.z);
    AVOID_END_POSE_XYZ = Eigen::Vector3d(AVOID_END_POSE_X + home_pose.pose.position.x,
                                         AVOID_END_POSE_Y + home_pose.pose.position.y,
                                         AVOID_END_POSE_Z + home_pose.pose.position.z);
    TRACE_START_POSE_XYZ = Eigen::Vector3d(TRACE_START_POSE_X + home_pose.pose.position.x,
                                           TRACE_START_POSE_Y + home_pose.pose.position.y,
                                           TRACE_START_POSE_Z + home_pose.pose.position.z);
    TRACE_END_POSE_XYZ = Eigen::Vector3d(TRACE_END_POSE_X + home_pose.pose.position.x,
                                         TRACE_END_POSE_Y + home_pose.pose.position.y,
                                         TRACE_END_POSE_Z + home_pose.pose.position.z);
    
    ROS_INFO(ANSI_COLOR_MAGENTA "[OK] Waypoints loaded relative to home (%.2f, %.2f, %.2f)" ANSI_COLOR_RESET, 
             home_pose.pose.position.x,
             home_pose.pose.position.y,
             home_pose.pose.position.z);
}

void MissionMaster::setPoint(Eigen::Vector3d pose_v3d)
{
    geometry_msgs::PoseStamped temp_pose;
    temp_pose.header.frame_id = "map";
    temp_pose.header.stamp = ros::Time::now();
    temp_pose.pose.position.x = pose_v3d.x();
    temp_pose.pose.position.y = pose_v3d.y();
    temp_pose.pose.position.z = pose_v3d.z();
    temp_pose.pose.orientation.x = 0.0;
    temp_pose.pose.orientation.y = 0.0;
    temp_pose.pose.orientation.z = 0.0;
    temp_pose.pose.orientation.w = 1.0;

    local_pos_pub.publish(temp_pose);
    
    // 降级为 DEBUG
    ROS_DEBUG("Setpoint: (%.2f, %.2f, %.2f)", pose_v3d.x(), pose_v3d.y(), pose_v3d.z());
}

bool MissionMaster::landExecute() {
    static bool mode_switched = false;
    
    // 检查当前模式是否为AUTO.LAND，若不是则切换
    if (current_vehicle_state_.mode != "AUTO.LAND" && !mode_switched) {
        mavros_msgs::SetMode set_mode_srv;
        set_mode_srv.request.custom_mode = "AUTO.LAND";

        if (set_mode_client.call(set_mode_srv) && set_mode_srv.response.mode_sent) {
            ROS_INFO(ANSI_COLOR_YELLOW "[LANDING] Switched to AUTO.LAND mode" ANSI_COLOR_RESET);
            mode_switched = true;
        } else {
            ROS_ERROR(ANSI_COLOR_RED "[FAIL] Failed to switch to AUTO.LAND mode" ANSI_COLOR_RESET);
            return false;
        }
    }

    // 检查当前高度是否低于航点容忍距离
    if (current_pose.pose.position.z <= TOLERANCE_WAYPOINT) {
        ROS_INFO(ANSI_COLOR_GREEN "[OK] Landing completed (height: %.2f m)" ANSI_COLOR_RESET, 
                 current_pose.pose.position.z);
        return true;
    } else {
        ROS_INFO_THROTTLE(2.0, "[LAND] Landing in progress (current height: %.2f m)", 
                         current_pose.pose.position.z);
        return false;
    }
}

// 暂时简化 Action 执行函数，直接返回成功
bool MissionMaster::pickExecute() {
    ROS_DEBUG("Pick action skipped (not integrated)");
    return true;
}

bool MissionMaster::avoidExecute() {
    ROS_DEBUG("Avoid action skipped (not integrated)");
    return true;
}

bool MissionMaster::traceExecute() {
    ROS_DEBUG("Trace action skipped (not integrated)");
    return true;
}

// 获取当前任务状态（用于测试/调试）
void MissionMaster::getState() {
    ROS_INFO_THROTTLE(1.0, "Current mission state: %d", static_cast<int>(current_mission_state_));
}
