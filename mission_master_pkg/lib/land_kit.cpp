/**
 * @file 降落相关
 */
#include "mission_master_pkg/mission_master.h"

/**
 * @brief 降落任务开始
 */
void MissionMaster::landStart()
{

    geometry_msgs::PoseStamped land_pose;
    land_pose.header.frame_id = "map";
    land_pose.header.stamp = ros::Time::now();
    land_pose.pose.position.x = home_pose.pose.position.x;
    land_pose.pose.position.y = home_pose.pose.position.y;
    land_pose.pose.position.z = 3.0; // 预留修改
    land_pose.pose.orientation.x = 0.0;
    land_pose.pose.orientation.y = 0.0;
    land_pose.pose.orientation.z = 0.0;
    land_pose.pose.orientation.w = 1.0;

    while (ros::ok())
    {
        setpoint_pub_.publish(land_pose);
        if (reachCheck(Eigen::Vector3d(home_pose.pose.position.x, home_pose.pose.position.y, 3.0)))
        {
            ROS_INFO("Arrived at Pickup Start Point");
            current_mission_state = EXECUTE_LAND_STATE;
            break;
        }

        ros::spinOnce();
        rate_.sleep();
    }
}

/**
 * @brief 降落执行
 */
void MissionMaster::landExecute()
{
    // 切autoland
    while (ros::ok())
    {
        // 检查当前模式是否为AUTO.LAND，若不是则切换
        if (current_state.mode != "AUTO.LAND")
        {
            mavros_msgs::SetMode land_setmode;
            land_setmode.request.custom_mode = "AUTO.LAND";
            if (set_mode_client_.call(land_setmode) && land_setmode.response.mode_sent)
            {
                ROS_INFO("AUTO.LAND enabled");
            }
        }
        else
        {
            ROS_INFO("Already in AUTO.LAND mode");
            current_mission_state = SUCCEED_LAND_STATE;
            break;
        }

        ros::spinOnce();
        rate_.sleep();
    }
}

/**
 * @brief 降落检查
 */
void MissionMaster::landCheck()
{
    //着陆检查并上锁
    while (ros::ok())
    {
        // 检查当前高度是否低于航点容忍距离（判断是否已着陆）
        if (current_pose.pose.position.z <= TOLERANCE_WAYPOINT)
        {
            ROS_INFO("Landed successfully");

            // 上锁
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = false; // 设置为上锁
            if (arming_client_.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle disarmed");
                current_mission_state = MISSION_SUCCEED_STATE;
                break;
            }
            else
            {
                ROS_ERROR("Failed to disarm vehicle");
            }
        }

        ros::spinOnce();
        rate_.sleep();
    }

}