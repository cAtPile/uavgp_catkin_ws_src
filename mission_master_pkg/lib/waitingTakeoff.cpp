#include "mission_master_pkg/mission_master.h"
/**
 * @brief 等待起飞
 */
void MissionMaster::waitingTakeoff()
{

    while (ros::ok())
    {

        if (current_vehicle_state_.mode == "OFFBOARD")
        {
            // 解锁armSet()
            current_mission_state = EXECUTE_TAKEOFF_STATE;
        }
        else
        {

            // 发送心跳信号
            local_pos_pub.publish(current_pose);
            ROS_INFO("Waiting for taking off..."); // 修改防止刷屏
        }
    }
}