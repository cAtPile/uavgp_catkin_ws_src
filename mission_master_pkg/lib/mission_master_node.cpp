/**
 * @file mission_master_node.cpp
 * @brief 
 * @date 2025/11/1
 */
#include "mission_master_pkg/mission_master_node.h"
#include <cmath>
#include <ros/console.h>

MissionMaster::MissionMaster() : rate_(20.0), current_mission_state_(ENUM_WATTING_TAKEOFF) {
    // 初始化订阅器
    state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &MissionMaster::stateCB, this);
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &MissionMaster::localPoseCB, this);

    // 初始化发布器
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    // 初始化服务客户端
    pickup_client = nh.serviceClient<apoc_pkg::pickup_service>("/pickup_service");
    avoid_client = nh.serviceClient<apoc_pkg::avoid_service>("/avoid_service");
    trace_client = nh.serviceClient<apoc_pkg::trace_service>("/trace_service");

    // 加载参数配置
    loadParams();

    ROS_INFO("MissionMaster initialized");
}

MissionMaster::~MissionMaster() {
    ROS_INFO("MissionMaster destroyed");
}

void MissionMaster::getState() {
    std::string state_str;
    switch (current_mission_state_) {
        case ENUM_WATTING_TAKEOFF: state_str = "Waiting takeoff"; break;
        case ENUM_TAKEOFF: state_str = "Taking off"; break;
        case ENUM_TAKEOFF_SUCCEED: state_str = "Takeoff succeeded"; break;
        case ENUM_FLYTO_PICKUP_POINT: state_str = "Flying to pickup point"; break;
        case ENUM_PICKUP_POINT: state_str = "Performing pickup"; break;
        case ENUM_PICKUP_SUCCEED: state_str = "Pickup succeeded"; break;
        case ENUM_FLYTO_AVOID_POINT: state_str = "Flying to avoid point"; break;
        case ENUM_AVOID_POINT: state_str = "Performing avoid"; break;
        case ENUM_AVOID_SUCCEED: state_str = "Avoid succeeded"; break;
        case ENUM_FLYTO_TRACE_POINT: state_str = "Flying to trace point"; break;
        case ENUM_TRACE_POINT: state_str = "Performing trace"; break;
        case ENUM_TRACE_SUCCEED: state_str = "Trace succeeded"; break;
        case ENUM_FLYTO_LAND_POINT: state_str = "Flying to land point"; break;
        case ENUM_LAND_POINT: state_str = "Landing"; break;
        case ENUM_LAND_SUCCEED: state_str = "Land succeeded"; break;
        default: state_str = "Unknown state";
    }

    ROS_INFO("Current mission state: %s", state_str.c_str());
    ROS_INFO("Vehicle state: %s", current_vehicle_state_.armed ? "Armed" : "Disarmed");
    ROS_INFO("Current position: (%.2f, %.2f, %.2f)",
             current_pose_.pose.position.x,
             current_pose_.pose.position.y,
             current_pose_.pose.position.z);
}

bool MissionMaster::isReachTarget(const geometry_msgs::PoseStamped& target_pose) {
    double dx = current_pose_.pose.position.x - target_pose.pose.position.x;
    double dy = current_pose_.pose.position.y - target_pose.pose.position.y;
    double dz = current_pose_.pose.position.z - target_pose.pose.position.z;
    double distance = sqrt(dx*dx + dy*dy + dz*dz);
    return distance < waypoint_tolerance_;
}

void MissionMaster::loadParams(){

    //
}