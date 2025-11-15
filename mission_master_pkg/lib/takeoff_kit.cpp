#include "mission_master_pkg/mission_master.h"
/**
 * @brief 等待起飞
 */
void MissionMaster::waitingTakeoff()
{

    while (ros::ok())
    {

        if (current_state.mode == "OFFBOARD")
        {
            ros::Time arm_start_time = ros::Time::now();
            if (armSet())
            {
                ROS_INFO("Switched to OFFBOARD mode and armed successfully");
                current_mission_state = EXECUTE_TAKEOFF_STATE;
                break;
            }
            else
            {
                ROS_WARN("Arming failed, retrying...");
                if (ros::Time::now() - arm_start_time > ros::Duration(10.0))
                {
                    ROS_ERROR("Arming timeout, please check the vehicle status");
                    current_mission_state = ERROR_STATE;

                    break;
                }
            }
        }
        else
        {

            // 发送心跳信号
            setpoint_pub_.publish(current_pose);
            ROS_INFO("Waiting for taking off...");
        }

        ros::spinOnce();
        rate_.sleep();
    }
}

/**
 * @brief 执行起飞
 */
void MissionMaster::takeoffExecute()
{
    home_pose = current_pose;
    loadWaypoints();

    setPoint(takeoff_waypoint_re);

    while (ros::ok())
    {
        setpoint_pub_.publish(temp_pose);
        if (reachCheck(takeoff_waypoint_re))
        {
            ROS_INFO("Takeoff set Success!");
            current_mission_state = SUCCEED_TAKEOFF_STATE;
            break;
        }
        ros::spinOnce();
        rate_.sleep();
    }
}

/**
 * @brief 起飞成功
 */
void MissionMaster::takeoffCheck()
{

    ROS_INFO("T check in");
    while (ros::ok())
    {
        ROS_INFO("T check loop");

        // 保持起飞定点
        setpoint_pub_.publish(temp_pose);

        if (reachCheck(takeoff_waypoint_re))
        {
            current_mission_state = START_PICKUP_STATE;
            ROS_INFO("Takeoff Success!");
            break;
        }

        ros::spinOnce();
        rate_.sleep();
    }
}

/**
 * @brief 解锁
 */
bool MissionMaster::armSet()
{

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if (arming_client_.call(arm_cmd) && arm_cmd.response.success)
    {
        ROS_INFO("Vehicle armed successfully");
        return true;
    }
    else
    {
        ROS_ERROR("Failed to arm vehicle");
        return false;
    }
}

/**
 * @brief 航点导入
 */
void MissionMaster::loadWaypoints()
{

    takeoff_waypoint_re = Eigen::Vector3d(
        takeoff_waypoint_v[0] + home_pose.pose.position.x,
        takeoff_waypoint_v[1] + home_pose.pose.position.y,
        takeoff_waypoint_v[2] + home_pose.pose.position.z);

    pickup_start_waypoint_re = Eigen::Vector3d(
        pickup_start_waypoint_v[0] + home_pose.pose.position.x,
        pickup_start_waypoint_v[1] + home_pose.pose.position.y,
        pickup_start_waypoint_v[2] + home_pose.pose.position.z);

    pickup_end_waypoint_re = Eigen::Vector3d(
        pickup_end_waypoint_v[0] + home_pose.pose.position.x,
        pickup_end_waypoint_v[1] + home_pose.pose.position.y,
        pickup_end_waypoint_v[2] + home_pose.pose.position.z);

    avoid_start_waypoint_re = Eigen::Vector3d(
        avoid_start_waypoint_v[0] + home_pose.pose.position.x,
        avoid_start_waypoint_v[1] + home_pose.pose.position.y,
        avoid_start_waypoint_v[2] + home_pose.pose.position.z);

    avoid_end_waypoint_re = Eigen::Vector3d(
        avoid_end_waypoint_v[0] + home_pose.pose.position.x,
        avoid_end_waypoint_v[1] + home_pose.pose.position.y,
        avoid_end_waypoint_v[2] + home_pose.pose.position.z);

    trace_start_waypoint_re = Eigen::Vector3d(
        trace_start_waypoint_v[0] + home_pose.pose.position.x,
        trace_start_waypoint_v[1] + home_pose.pose.position.y,
        trace_start_waypoint_v[2] + home_pose.pose.position.z);

    trace_end_waypoint_re = Eigen::Vector3d(
        trace_end_waypoint_v[0] + home_pose.pose.position.x,
        trace_end_waypoint_v[1] + home_pose.pose.position.y,
        trace_end_waypoint_v[2] + home_pose.pose.position.z);
        
}

