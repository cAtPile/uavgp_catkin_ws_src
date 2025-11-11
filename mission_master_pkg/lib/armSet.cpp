#include "mission_master_pkg/mission_master.h"
/**
 * @brief 解锁
 */
void MissionMaster::armSet()
{
    //超时检查 
    // 解锁无人机
    ROS_INFO("Attempting to arm the vehicle...");

    // 创建解锁服务请求
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true; // true表示解锁，false表示上锁
    arming_client.call(arm_cmd);
}