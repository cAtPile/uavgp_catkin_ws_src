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
            ros::Time arm_start_time = ros::Time::now();
            if (armSet())
            {
                ROS_INFO("Switched to OFFBOARD mode and armed successfully");
                current_mission_state = EXECUTE_TAKEOFF_STATE;
                break;
            }
            else
            {
                ROS_WARN("Arming failed, retrying...");
                if(ros::Time::now() - arm_start_time > ros::Duration(10.0))
                {
                    ROS_ERROR("Arming timeout, please check the vehicle status");
                    current_mission_state = ERROR_ARM_STATE;

                    break;
                }
            }
        }
        else
        {

            // 发送心跳信号
            local_pos_pub.publish(current_pose);
            ROS_INFO("Waiting for taking off...");
        }

        ros::spinOnce();
        rate_.sleep();
    }
}