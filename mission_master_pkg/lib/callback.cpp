/**
 * @file callback.cpp
 * @brief 回调函数
 * @date 2025/11/2
 */
#include <mission_master_pkg/mission_master.h>

void MissionMaster::localPoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
}

void MissionMaster::stateCheckCB(const mavros_msgs::State::ConstPtr &msg)
{

    // 记录无人机状态
    current_vehicle_state_ = *msg;

    // 判断解锁
    if (current_vehicle_state_.armed)
    {

        // 判断当前状态
        switch (current_mission_state_)
        {

        case ENUM_WATTING_TAKEOFF:
            loadWaypoints();
            setPoint(TAKEOFF_POSE_XYZ);
            current_mission_state_ = ENUM_TAKEOFF;
            break;

        case ENUM_TAKEOFF:
            setPoint(TAKEOFF_POSE_XYZ);
            if (reachCheck(TAKEOFF_POSE_XYZ))
                current_mission_state_ = ENUM_TAKEOFF_SUCCEED;
            break;

        case ENUM_TAKEOFF_SUCCEED:
            setPoint(PICKUP_START_POSE_XYZ);
            current_mission_state_ = ENUM_FLYTO_PICKUP_POINT;
            break;

        case ENUM_FLYTO_PICKUP_POINT:
            setPoint(PICKUP_START_POSE_XYZ);
            if (reachCheck(PICKUP_START_POSE_XYZ))
                current_mission_state_ = ENUM_PICKUP_POINT;
            break;

        case ENUM_PICKUP_POINT:
            if (pickExecute())
                current_mission_state_ = ENUM_PICKUP_SUCCEED;
            break;

        case ENUM_PICKUP_SUCCEED:
            setPoint(PICKUP_END_POSE_XYZ);
            if (reachCheck(PICKUP_END_POSE_XYZ))
                current_mission_state_ = ENUM_FLYTO_AVOID_POINT;
            break;

        case ENUM_FLYTO_AVOID_POINT:
            setPoint(AVOID_START_POSE_XYZ);
            if (reachCheck(AVOID_START_POSE_XYZ))
                current_mission_state_ = ENUM_AVOID_POINT;
            break;

        case ENUM_AVOID_POINT:
            if (avoidExecute())
                current_mission_state_ = ENUM_AVOID_SUCCEED;
            break;

        case ENUM_AVOID_SUCCEED:
            setPoint(AVOID_END_POSE_XYZ);
            if (reachCheck(AVOID_END_POSE_XYZ))
                current_mission_state_ = ENUM_FLYTO_TRACE_POINT;
            break;

        case ENUM_FLYTO_TRACE_POINT:
            setPoint(TRACE_START_POSE_XYZ);
            if (reachCheck(TRACE_START_POSE_XYZ)) current_mission_state_ = ENUM_TRACE_POINT;
            break;

        case ENUM_TRACE_POINT:
            if (traceExecute())
                current_mission_state_ = ENUM_TRACE_SUCCEED;
            break;

        case ENUM_TRACE_SUCCEED:
            setPoint(TRACE_END_POSE_XYZ);
            if (reachCheck(TRACE_END_POSE_XYZ))
                current_mission_state_ = ENUM_FLYTO_LAND_POINT;
            break;

        case ENUM_FLYTO_LAND_POINT:
            setPoint(TAKEOFF_POSE_XYZ);
            if (reachCheck(TAKEOFF_POSE_XYZ))
                current_mission_state_ = ENUM_LAND_POINT;
            break;

        case ENUM_LAND_POINT:
            if (landExecute())
                current_mission_state_ = ENUM_LAND_SUCCEED;
            break;

        case ENUM_LAND_SUCCEED:
            // 切降落
            ROS_INFO("Mission Succeed!!");
            break;

        default:
            // local_pos_pub.publish(trace_start_pose);
            break;
        }
    }else{
        // 解锁无人机
        ROS_INFO("Attempting to arm the vehicle...");

        // 创建解锁服务请求
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;  // true表示解锁，false表示上锁

        // 发送解锁请求（等待服务响应，超时5秒）
        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
            ROS_INFO("Vehicle armed successfully!");
        } else {
            ROS_ERROR("Failed to arm the vehicle!");
    }
    }
}
