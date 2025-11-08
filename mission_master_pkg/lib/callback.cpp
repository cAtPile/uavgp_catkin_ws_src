/**
 * @file callback.cpp
 * @brief 回调函数
 * @date 2025/11/2
 * @note 优化日志输出，使用颜色标记重要信息
 */
#include <mission_master_pkg/mission_master.h>

// ANSI 颜色代码
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

void MissionMaster::localPoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
}

void MissionMaster::stateCheckCB(const mavros_msgs::State::ConstPtr &msg)
{
    // 记录无人机状态变化
    bool state_changed = (current_vehicle_state_.mode != msg->mode || 
                         current_vehicle_state_.armed != msg->armed);
    current_vehicle_state_ = *msg;

    // 状态改变时输出
    if (state_changed) {
        ROS_INFO(ANSI_COLOR_CYAN "====================================" ANSI_COLOR_RESET);
        ROS_INFO(ANSI_COLOR_CYAN "Vehicle: %s | Mode: %s" ANSI_COLOR_RESET,
                 current_vehicle_state_.armed ? "✈ ARMED" : "⊗ DISARMED",
                 current_vehicle_state_.mode.c_str());
        ROS_INFO(ANSI_COLOR_CYAN "====================================" ANSI_COLOR_RESET);
    }

    // 等待 OFFBOARD 准备完成
    if (!offboard_ready_) {
        return;
    }

    // 判断是否在 OFFBOARD 模式
    if (current_vehicle_state_.mode == "OFFBOARD")
    {
        // 判断当前任务状态
        switch (current_mission_state_)
        {
        case ENUM_WATTING_TAKEOFF:
        {
            loadWaypoints();
            
            // 解锁无人机
            if (!current_vehicle_state_.armed) {
                ROS_INFO(ANSI_COLOR_YELLOW "[ARM] Arming vehicle..." ANSI_COLOR_RESET);
                mavros_msgs::CommandBool arm_cmd;
                arm_cmd.request.value = true;
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO(ANSI_COLOR_GREEN "[OK] Vehicle ARMED!" ANSI_COLOR_RESET);
                } else {
                    ROS_WARN(ANSI_COLOR_RED "[FAIL] ARM failed, retrying..." ANSI_COLOR_RESET);
                }
            }
            
            setPoint(TAKEOFF_POSE_XYZ);
            current_mission_state_ = ENUM_TAKEOFF;
            ROS_INFO(ANSI_COLOR_GREEN "====================================" ANSI_COLOR_RESET);
            ROS_INFO(ANSI_COLOR_GREEN "[DRONE] MISSION START: TAKEOFF" ANSI_COLOR_RESET);
            ROS_INFO(ANSI_COLOR_GREEN "   Target: (%.2f, %.2f, %.2f)" ANSI_COLOR_RESET,
                     TAKEOFF_POSE_XYZ.x(), TAKEOFF_POSE_XYZ.y(), TAKEOFF_POSE_XYZ.z());
            ROS_INFO(ANSI_COLOR_GREEN "====================================" ANSI_COLOR_RESET);
            break;
        }

        case ENUM_TAKEOFF:
            setPoint(TAKEOFF_POSE_XYZ);
            if (reachCheck(TAKEOFF_POSE_XYZ)) {
                ROS_INFO(ANSI_COLOR_GREEN "[OK] Takeoff complete!" ANSI_COLOR_RESET);
                current_mission_state_ = ENUM_TAKEOFF_SUCCEED;
            }
            break;

        case ENUM_TAKEOFF_SUCCEED:
            setPoint(PICKUP_START_POSE_XYZ);
            current_mission_state_ = ENUM_FLYTO_PICKUP_POINT;
            ROS_INFO(ANSI_COLOR_BLUE "====================================" ANSI_COLOR_RESET);
            ROS_INFO(ANSI_COLOR_BLUE "[WP] Waypoint 1: (%.2f, %.2f, %.2f)" ANSI_COLOR_RESET,
                     PICKUP_START_POSE_XYZ.x(), PICKUP_START_POSE_XYZ.y(), PICKUP_START_POSE_XYZ.z());
            break;

        case ENUM_FLYTO_PICKUP_POINT:
            setPoint(PICKUP_START_POSE_XYZ);
            if (reachCheck(PICKUP_START_POSE_XYZ)) {
                ROS_INFO(ANSI_COLOR_GREEN "[OK] Waypoint 1 reached" ANSI_COLOR_RESET);
                current_mission_state_ = ENUM_PICKUP_POINT;
            }
            break;

        case ENUM_PICKUP_POINT:
            // 跳过 PICK Action
            ROS_INFO(ANSI_COLOR_YELLOW "[SKIP] [SKIP] Pickup action" ANSI_COLOR_RESET);
            current_mission_state_ = ENUM_PICKUP_SUCCEED;
            break;

        case ENUM_PICKUP_SUCCEED:
            setPoint(PICKUP_END_POSE_XYZ);
            if (reachCheck(PICKUP_END_POSE_XYZ)) {
                ROS_INFO(ANSI_COLOR_GREEN "[OK] Waypoint 2 reached" ANSI_COLOR_RESET);
                current_mission_state_ = ENUM_FLYTO_AVOID_POINT;
            }
            break;

        case ENUM_FLYTO_AVOID_POINT:
            setPoint(AVOID_START_POSE_XYZ);
            if (reachCheck(AVOID_START_POSE_XYZ)) {
                ROS_INFO(ANSI_COLOR_GREEN "[OK] Waypoint 3 reached" ANSI_COLOR_RESET);
                current_mission_state_ = ENUM_AVOID_POINT;
            }
            break;

        case ENUM_AVOID_POINT:
            // 跳过 AVOID Action
            ROS_INFO(ANSI_COLOR_YELLOW "[SKIP] [SKIP] Avoid action" ANSI_COLOR_RESET);
            current_mission_state_ = ENUM_AVOID_SUCCEED;
            break;

        case ENUM_AVOID_SUCCEED:
            setPoint(AVOID_END_POSE_XYZ);
            if (reachCheck(AVOID_END_POSE_XYZ)) {
                ROS_INFO(ANSI_COLOR_GREEN "[OK] Waypoint 4 reached" ANSI_COLOR_RESET);
                current_mission_state_ = ENUM_FLYTO_TRACE_POINT;
            }
            break;

        case ENUM_FLYTO_TRACE_POINT:
            setPoint(TRACE_START_POSE_XYZ);
            if (reachCheck(TRACE_START_POSE_XYZ)) {
                ROS_INFO(ANSI_COLOR_GREEN "[OK] Waypoint 5 reached" ANSI_COLOR_RESET);
                current_mission_state_ = ENUM_TRACE_POINT;
            }
            break;

        case ENUM_TRACE_POINT:
            // 跳过 TRACE Action
            ROS_INFO(ANSI_COLOR_YELLOW "[SKIP] [SKIP] Trace action" ANSI_COLOR_RESET);
            current_mission_state_ = ENUM_TRACE_SUCCEED;
            break;

        case ENUM_TRACE_SUCCEED:
            setPoint(TRACE_END_POSE_XYZ);
            if (reachCheck(TRACE_END_POSE_XYZ)) {
                ROS_INFO(ANSI_COLOR_GREEN "[OK] Waypoint 6 reached" ANSI_COLOR_RESET);
                current_mission_state_ = ENUM_FLYTO_LAND_POINT;
            }
            break;

        case ENUM_FLYTO_LAND_POINT:
            setPoint(TAKEOFF_POSE_XYZ);
            if (reachCheck(TAKEOFF_POSE_XYZ)) {
                ROS_INFO(ANSI_COLOR_MAGENTA "====================================" ANSI_COLOR_RESET);
                ROS_INFO(ANSI_COLOR_MAGENTA "[HOME] Returned to HOME position" ANSI_COLOR_RESET);
                ROS_INFO(ANSI_COLOR_MAGENTA "[LAND] Initiating landing..." ANSI_COLOR_RESET);
                ROS_INFO(ANSI_COLOR_MAGENTA "====================================" ANSI_COLOR_RESET);
                current_mission_state_ = ENUM_LAND_POINT;
            }
            break;

        case ENUM_LAND_POINT:
            // 调用降落函数
            if (landExecute()) {
                current_mission_state_ = ENUM_LAND_SUCCEED;
            }
            break;

        case ENUM_LAND_SUCCEED:
            // 任务完成，只输出一次
            ROS_INFO_ONCE(ANSI_COLOR_GREEN "=════════════════════════════════════╗" ANSI_COLOR_RESET);
            ROS_INFO_ONCE(ANSI_COLOR_GREEN "|                                    |" ANSI_COLOR_RESET);
            ROS_INFO_ONCE(ANSI_COLOR_GREEN "|  [OK][OK][OK] MISSION COMPLETED! [OK][OK][OK]       |" ANSI_COLOR_RESET);
            ROS_INFO_ONCE(ANSI_COLOR_GREEN "|                                    |" ANSI_COLOR_RESET);
            ROS_INFO_ONCE(ANSI_COLOR_GREEN "=════════════════════════════════════╝" ANSI_COLOR_RESET);
            // 任务完成后不再执行
            break;

        default:
            break;
        }
    }
    else
    {
        // 降落模式检查：如果在 AUTO.LAND 模式且高度低于阈值，认为降落完成
        if (current_vehicle_state_.mode == "AUTO.LAND" && 
            current_mission_state_ == ENUM_LAND_POINT) {
            
            if (current_pose.pose.position.z <= TOLERANCE_WAYPOINT && !current_vehicle_state_.armed) {
                ROS_INFO(ANSI_COLOR_GREEN "[OK] Landing completed (Disarmed at %.2f m)" ANSI_COLOR_RESET,
                         current_pose.pose.position.z);
                current_mission_state_ = ENUM_LAND_SUCCEED;
                
                // 输出任务完成
                ROS_INFO(ANSI_COLOR_GREEN "=════════════════════════════════════╗" ANSI_COLOR_RESET);
                ROS_INFO(ANSI_COLOR_GREEN "|                                    |" ANSI_COLOR_RESET);
                ROS_INFO(ANSI_COLOR_GREEN "|  [OK][OK][OK] MISSION COMPLETED! [OK][OK][OK]       |" ANSI_COLOR_RESET);
                ROS_INFO(ANSI_COLOR_GREEN "|                                    |" ANSI_COLOR_RESET);
                ROS_INFO(ANSI_COLOR_GREEN "=════════════════════════════════════╝" ANSI_COLOR_RESET);
            } else {
                // 降落中，限流输出
                ROS_INFO_THROTTLE(3.0, "[LAND] Landing... (Height: %.2f m | %s)", 
                                 current_pose.pose.position.z,
                                 current_vehicle_state_.armed ? "Armed" : "Disarmed");
            }
        } 
        // 其他非 OFFBOARD 模式，每 5 秒提示一次
        else if (current_mission_state_ != ENUM_LAND_SUCCEED) {
            ROS_WARN_THROTTLE(5.0, "[WARN] Not in OFFBOARD mode (Current: %s)", 
                             current_vehicle_state_.mode.c_str());
        }
    }
}
