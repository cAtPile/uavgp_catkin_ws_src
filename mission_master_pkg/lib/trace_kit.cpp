/**
 * @file trace_kit.cpp
 * @brief 跟踪相关
 */
#include "mission_master_pkg/mission_master.h"、

/**
 * @brief 跟踪任务开始
 */
void MissionMaster::traceStart()
{

    setPoint(TRACE_START_WAYPOINT);

    while (ros::ok())
    {
        setpoint_pub_.publish(temp_pose);
        if (reachCheck(TRACE_START_WAYPOINT))
        {
            ROS_INFO("Arrived at Pickup Start Point");
            current_mission_state = EXECUTE_TRACE_STATE;
            break;
        }

        ros::spinOnce();
        rate_.sleep();
    }
}

/**
 * @brief 跟踪执行
 */
void MissionMaster::traceExecute()
{
    // 预留
    current_mission_state = SUCCEED_TRACE_STATE;
}

/**
 * @brief 跟踪检查
 */
void MissionMaster::traceCheck()
{
    setPoint(TRACE_END_WAYPOINT);
    while (ros::ok())
    {
        setpoint_pub_.publish(temp_pose);
        if (reachCheck(TRACE_END_WAYPOINT))
        {
            current_mission_state = START_LAND_STATE;
        }

        ros::spinOnce();
        rate_.sleep();
    }
}