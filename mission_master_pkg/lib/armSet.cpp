#include "mission_master_pkg/mission_master.h"
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