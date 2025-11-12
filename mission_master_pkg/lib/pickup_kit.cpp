#include"mission_master_pkg/mission_master.h"
/**
 * @brief 抓取开始点
 */
void MissionMaster::pickupStart(){

    setPoint(PICKUP_START_WAYPOINT);

    while(ros::ok()){
        setpoint_pub_.publish(temp_pose);
        if(reachCheck(TOLERANCE_WAYPOINT)){
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
    //预留
    current_mission_state = SUCCEED_PICKUP_STATE;
}

/**
 * @brief 抓取任务结束检查
 */
void MissionMaster::pickupCheck()
{
    //setPoint(PICKUP_END_WAYPOINT);
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