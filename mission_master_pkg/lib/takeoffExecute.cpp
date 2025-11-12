#include "mission_master_pkg/mission_master.h"
/**
 * @brief 执行起飞
 */
void MissionMaster::takeoffExecute()
{
    setPoint(TAKEOFF_WAYPOINT);

    while (ros::ok())
    {
        setpoint_pub_.publish(temp_pose);
        if (reachCheck(TOLERANCE_WAYPOINT)){
            ROS_INFO("Takeoff Success!");
            current_mission_state = SUCCEED_TAKEOFF_STATE;
            break;
        }else{
            //超时检查预留
        }

        ros::spinOnce();
        rate_.sleep();
    }
}