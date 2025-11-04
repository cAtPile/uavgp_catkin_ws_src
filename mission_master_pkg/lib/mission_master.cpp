/**
 * @file mission_master_node.cpp
 * @brief
 * @date 2025/11/1
 */
#include "mission_master_pkg/mission_master.h"
#include <cmath>
#include <ros/console.h>

MissionMaster::MissionMaster() : rate_(20.0), 
                                 current_mission_state_(ENUM_WATTING_TAKEOFF),
                                 pick_clientor("/pick_server/pick", true),
                                 trace_clientor("/trace_server/trace", true),
                                 avoid_clientor("/avoid_server/avoid", true)
{
    // 初始化订阅器
    state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &MissionMaster::stateCB, this);
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &MissionMaster::localPoseCB, this);

    // 初始化发布器
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    // 加载参数配置
    loadParams();

    // 初始化模式设置客户端
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
}

MissionMaster::~MissionMaster()
{
    ROS_INFO("MissionMaster destroyed");
}

void MissionMaster::getState()
{
    std::string state_str;
    switch (current_mission_state_)
    {
    case ENUM_WATTING_TAKEOFF:
        state_str = "Waiting takeoff";
        break;
    case ENUM_TAKEOFF:
        state_str = "Taking off";
        break;
    case ENUM_TAKEOFF_SUCCEED:
        state_str = "Takeoff succeeded";
        break;
    case ENUM_FLYTO_PICKUP_POINT:
        state_str = "Flying to pickup point";
        break;
    case ENUM_PICKUP_POINT:
        state_str = "Performing pickup";
        break;
    case ENUM_PICKUP_SUCCEED:
        state_str = "Pickup succeeded";
        break;
    case ENUM_FLYTO_AVOID_POINT:
        state_str = "Flying to avoid point";
        break;
    case ENUM_AVOID_POINT:
        state_str = "Performing avoid";
        break;
    case ENUM_AVOID_SUCCEED:
        state_str = "Avoid succeeded";
        break;
    case ENUM_FLYTO_TRACE_POINT:
        state_str = "Flying to trace point";
        break;
    case ENUM_TRACE_POINT:
        state_str = "Performing trace";
        break;
    case ENUM_TRACE_SUCCEED:
        state_str = "Trace succeeded";
        break;
    case ENUM_FLYTO_LAND_POINT:
        state_str = "Flying to land point";
        break;
    case ENUM_LAND_POINT:
        state_str = "Landing";
        break;
    case ENUM_LAND_SUCCEED:
        state_str = "Land succeeded";
        break;
    default:
        state_str = "Unknown state";
    }

    ROS_INFO("Current mission state: %s", state_str.c_str());
    ROS_INFO("Vehicle state: %s", current_vehicle_state_.armed ? "Armed" : "Disarmed");
    ROS_INFO("Current position: (%.2f, %.2f, %.2f)",
             current_pose_.pose.position.x,
             current_pose_.pose.position.y,
             current_pose_.pose.position.z);
}

bool MissionMaster::reachCheck(Eigen::Vector3d pose_v3d)
{
    double dx = current_pose_.pose.position.x - pose_v3d.x();
    double dy = current_pose_.pose.position.y - pose_v3d.y();
    double dz = current_pose_.pose.position.z - pose_v3d.z();
    double distance = sqrt(dx * dx + dy * dy + dz * dz);
    return distance < waypoint_tolerance_;
}

void MissionMaster::loadParams()
{
    //========飞行参数
    nh.param("tolerance_waypoint", TOLERANCE_WAYPOINT, 0.03);

    //========航点参数
    nh.param("takeoff_pose_x", TAKEOFF_POSE_X, 0.0);
    nh.param("takeoff_pose_y", TAKEOFF_POSE_Y, 0.0);
    nh.param("takeoff_pose_z", TAKEOFF_POSE_Z, 0.0);
    nh.param("pickup_start_pose_x", PICKUP_START_POSE_X, 0.0);
    nh.param("pickup_start_pose_y", PICKUP_START_POSE_Y, 0.0);
    nh.param("pickup_start_pose_z", PICKUP_START_POSE_Z, 0.0);
    nh.param("pickup_end_pose_x", PICKUP_END_POSE_X, 0.0);
    nh.param("pickup_end_pose_y", PICKUP_END_POSE_Y, 0.0);
    nh.param("pickup_end_pose_z", PICKUP_END_POSE_Z, 0.0);
    nh.param("avoid_start_pose_x", AVOID_START_POSE_X, 0.0);
    nh.param("avoid_start_pose_y", AVOID_START_POSE_Y, 0.0);
    nh.param("avoid_start_pose_z", AVOID_START_POSE_Z, 0.0);
    nh.param("avoid_end_pose_x", AVOID_END_POSE_X, 0.0);
    nh.param("avoid_end_pose_y", AVOID_END_POSE_Y, 0.0);
    nh.param("avoid_end_pose_z", AVOID_END_POSE_Z, 0.0);
    nh.param("trace_start_pose_x", TRACE_START_POSE_X, 0.0);
    nh.param("trace_start_pose_y", TRACE_START_POSE_Y, 0.0);
    nh.param("trace_start_pose_z", TRACE_START_POSE_Z, 0.0);
    nh.param("trace_end_pose_x", TRACE_END_POSE_X, 0.0);
    nh.param("trace_end_pose_y", TRACE_END_POSE_Y, 0.0);
    nh.param("trace_end_pose_z", TRACE_END_POSE_Z, 0.0);
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
}

bool MissionMaster::landExecute() {
    // 检查当前模式是否为AUTO.LAND，若不是则切换
    if (current_vehicle_state_.mode != "AUTO.LAND") {
        mavros_msgs::SetMode set_mode_srv;
        set_mode_srv.request.custom_mode = "AUTO.LAND"; // 设置为自动降落模式

        // 调用服务切换模式
        if (set_mode_client.call(set_mode_srv) && set_mode_srv.response.mode_sent) {
            ROS_INFO("Successfully switched to AUTO.LAND mode");
        } else {
            ROS_ERROR("Failed to switch to AUTO.LAND mode");
            return false; // 模式切换失败，返回false
        }
    }

    // 检查当前高度是否低于航点容忍距离（判断是否已着陆）
    // 假设高度小于等于容忍距离时视为着陆成功
    if (current_pose.pose.position.z <= TOLERANCE_WAYPOINT) {
        ROS_INFO("Landing completed (height: %.2f m)", current_pose.pose.position.z);
        return true;
    } else {
        ROS_INFO("Landing in progress (current height: %.2f m)", current_pose.pose.position.z);
        return false;
    }
}