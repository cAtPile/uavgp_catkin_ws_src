#include "mission_master_pkg/mission_master.h"
/**
 * @brief 抓取开始点
 */
void MissionMaster::pickupStart()
{
    ROS_INFO("P start");

    setPoint(PICKUP_START_WAYPOINT);

    while (ros::ok())
    {
        ROS_INFO("P loop");

        setpoint_pub_.publish(temp_pose);
        if (reachCheck(PICKUP_START_WAYPOINT))
        {
            ROS_INFO("Arrived at Pickup Start Point");
            current_mission_state = EXECUTE_PICKUP_STATE;
            break;
        }

        ros::spinOnce();
        rate_.sleep();
    }
}

/**
 * @brief 抓取执行
 */
void MissionMaster::pickupExecute()
{
    ROS_INFO("EXE PICKUP");

    pickLoop();
    temp_pose.pose.position.x = currernt_pose.pose.position.x;
    temp_pose.pose.position.y = currernt_pose.pose.position.y;
    temp_pose.pose.position.z = 5.0 + home_pose.pose.position.z;

    while (ros::ok())
    {

        setpoint_pub_.publish(temp_pose);
        if (reachCheck(Eigen::Vector3d(temp_pose.pose.position.x, temp_pose.pose.position.y, temp_pose.pose.position.z)))
        {
            current_mission_state = SUCCEED_PICKUP_STATE;
            break;
        }
    }

    // 预留
}

/**
 * @brief 抓取任务结束检查
 */
void MissionMaster::pickupCheck()
{
    ROS_INFO("EXE PICK END");

    setPoint(PICKUP_END_WAYPOINT);
    while (ros::ok())
    {
        ROS_INFO("EXE PICKUP END LOOP ");

        setpoint_pub_.publish(temp_pose);
        if (reachCheck(PICKUP_END_WAYPOINT))
        {
            ROS_INFO("EXE PICKUP END CHECK ");

            current_mission_state = START_AVOID_STATE;
            break;
        }
    }
}

pickLoop()
{
    ROS_INFO("PICK LOOP IN")

    // pick_pose 初始化
    geometry_msgs::PoseStamped pick_pose;
    pick_pose.header.frame_id = "map";
    pick_pose.pose.orientation.x = 0;
    pick_pose.pose.orientation.y = 0;
    pick_pose.pose.orientation.z = 0;
    pick_pose.pose.orientation.w = 1;

    double ball_x, ball_y;
    double track_center_x, track_center_y;
    double rel_cam_x, rel_cam_y;
    double local_ball_x, loacl_ball_y;
    double via_x, via_y, via_z;
    double cam_loc_rate;
    double aim_high;
    double pick_time;
    double pick_land_vel;

    ros::time loop_start_time = ros::Time::now();
    // 抓取主循环
    while (ros::ok())
    {
        ROS_INFO("PICK LOOP pose");

        // 获取像素坐标
        ball_x = current_camtrack.ball_x;
        ball_y = current_camtrack.ball_y;

        // 像素相对化
        rel_cam_x = ball_x - track_center_x;
        rel_cam_y = ball_y - track_center_y;

        // 转化为实际坐标
        rel_cam_x = rel_cam_x * cam_loc_rate;
        rel_cam_y = rel_cam_y * cam_loc_rate;

        // 计算目标点
        via_x = current_pose.pose.position.x - rel_cam_x;
        via_y = current_pose.pose.position.y - rel_cam_y;

        // 计算高度via_z
        pick_time = (ros::Time::now() - loop_start_time).toSec();
        via_z = pick_land_vel * pick_time;

        // 构造点
        pick_pose.pose.position.x = via_x;
        pick_pose.pose.position.y = via_y;
        pick_pose.pose.position.z = via_z;

        // 发送飞行指令
        setpoint_pub_.publish(pick_pose);

        // 到达目标高度
        if (current_pose.pose.position.z <= aim_high)
        {
            gripPick();
        }

        if (current_camtrack.in_gripper == true)
        {
            // 跳出抓取循环
            ROS_INFO("PICK LOOP out");

            break;
        }

        ros::spinOnce();
        rate_.sleep();
    }
}

gripPick()
{
}
