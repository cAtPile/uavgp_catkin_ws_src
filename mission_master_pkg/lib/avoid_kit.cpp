/**
 * @file 避障相关
 */
#include "mission_master_pkg/mission_master.h"

/**
 * @brief 避障任务开始
 */
void MissionMaster::avoidStart()
{

    setPoint(AVOID_START_WAYPOINT);

    while (ros::ok())
    {
        setpoint_pub_.publish(temp_pose);
        if (reachCheck(AVOID_START_WAYPOINT))
        {
            ROS_INFO("Arrived at Pickup Start Point");
            current_mission_state = EXECUTE_AVOID_STATE;
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
    // 预留
    current_mission_state = SUCCEED_AVOID_STATE;
}

/**
 * @brief 避障检查
 */
void MissionMaster::avoidCheck()
{
    setPoint(AVOID_END_WAYPOINT);
    while (ros::ok())
    {
        setpoint_pub_.publish(temp_pose);
        if (reachCheck(AVOID_END_WAYPOINT))
        {
            current_mission_state = START_TRACE_STATE;
        }

        ros::spinOnce();
        rate_.sleep();
    }
}