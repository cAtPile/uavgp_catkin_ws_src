#include "mission_master_pkg/mission_master.h"
/**
 * @brief 抓取任务结束检查
 */
void MissionMaster::pickupCheck()
{
    setPoint(PICKUP_END_WAYPOINT);
    while (ros::ok())
    {
        setpoint_pub_.publish(temp_pose);
        if (reachCheck(PICKUP_END_WAYPOINT))
        {
            current_mission_state = START_AVOID_STATE;
        }

        ros::spinOnce();
        rate_.sleep();
    }
}