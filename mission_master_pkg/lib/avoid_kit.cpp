/**
 * @file 避障相关
 */
#include "mission_master_pkg/mission_master.h"

/**
 * @brief 避障任务开始
 */
void MissionMaster::avoidStart()
{
    ROS_DEBUG("A start");

    setPoint(avoid_start_waypoint_re);

    while (ros::ok())
    {
        ROS_DEBUG("A Loop");
        setpoint_pub_.publish(temp_pose);
        if (reachCheck(avoid_start_waypoint_re))
        {
            ROS_INFO("Arrived at avoid Start Point");
            current_mission_state = mission_queue[mission_queue_index];
            mission_queue_index++;
            break;
        }

        ros::spinOnce();
        rate_.sleep();
    }
}

/**
 * @brief 避障执行
 */
void MissionMaster::avoidExecute()
{
    ROS_INFO("A exe");
    avoidWaypointsLoad();
    avoidWaypointsLoop();

    // 预留
    current_mission_state = mission_queue[mission_queue_index];
    mission_queue_index++;
}

/**
 * @brief 避障检查
 */
void MissionMaster::avoidCheck()
{
    ROS_DEBUG("A check");

    setPoint(avoid_end_waypoint_re);
    while (ros::ok())
    {
        ROS_DEBUG("Ac Loop");

        setpoint_pub_.publish(temp_pose);
        if (reachCheck(avoid_end_waypoint_re))
        {
            current_mission_state = mission_queue[mission_queue_index];
            mission_queue_index++;
            setpoint_pub_.publish(current_pose);

            break;
        }

        ros::spinOnce();
        rate_.sleep();
    }
}

// 避障点压入
void MissionMaster::avoidWaypointsLoad()
{
    std::vector<Eigen::Vector3d> avoid_waypoints_v3d;
    int wap_av_size = waypoints_group_x.size();
    for (int i = 0; i < wap_av_size; i++)
    {

        double waypoint_x = waypoints_group_x[i];
        double waypoint_y = waypoints_group_y[i];
        double waypoint_z = waypoints_group_[i];
        avoid_waypoints_v3d.emplace_back(waypoint_x, waypoint_y, waypoint_z);
    }
}

// 航点避障循环
void MissionMaster::avoidWaypointsLoop()
{
    int wap_av_v3d_size = avoid_waypoints_v3d.size();
    for (int i = 0; i < wap_av_v3d_size; i++)
    {
        Eigen::Vector3d wap_av_v3d_temp = avoid_waypoints_v3d[i];
        while (reachCheck(wap_av_v3d_temp))
        {
            setPoint(wap_av_v3d_temp);
            setpoint_pub_.publish(temp_pose);

            ros::spinOnce();
            rate_.sleep();
        }
    }
}
