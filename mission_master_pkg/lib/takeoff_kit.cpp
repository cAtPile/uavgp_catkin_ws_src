#include "mission_master_pkg/mission_master.h"
/**
 * @brief 等待起飞
 */
void MissionMaster::waitingTakeoff()
{

    while (ros::ok())
    {

        if (current_state.mode == "OFFBOARD")
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
                    current_mission_state = ERROR_STATE;

                    break;
                }
            }
        }
        else
        {

            // 发送心跳信号
            setpoint_pub_.publish(current_pose);
            ROS_INFO("Waiting for taking off...");
        }

        ros::spinOnce();
        rate_.sleep();
    }
}

/**
 * @brief 执行起飞
 */
void MissionMaster::takeoffExecute()
{
    home_pose = current_pose;
    loadWaypoints();
    
    setPoint(TAKEOFF_WAYPOINT);

    while (ros::ok())
    {
        setpoint_pub_.publish(temp_pose);
        if (reachCheck(TAKEOFF_WAYPOINT)){
            ROS_INFO("Takeoff set Success!");
            current_mission_state = SUCCEED_TAKEOFF_STATE;
            break;
        }else{
            //超时检查预留
        }

        ros::spinOnce();
        rate_.sleep();
    }
}

/**
 * @brief 起飞成功
 */
void MissionMaster::takeoffCheck(){

    ROS_INFO("Takeoff check !");


    while(ros::ok()){

        //保持起飞定点
        setpoint_pub_.publish(temp_pose);

            ROS_INFO("Takeoff ing ");


        if(reachCheck(TAKEOFF_WAYPOINT)){
            current_mission_state = SUCCEED_TAKEOFF_STATE;
            ROS_INFO("Takeoff Success!");
            break;
        }
    }

}

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

/**
 * @brief 航点导入
 */
void MissionMaster::loadWaypoints(){

    TAKEOFF_WAYPOINT = Eigen::Vector3d(TAKEOFF_POSE_X + home_pose.pose.position.x,
                                       TAKEOFF_POSE_Y + home_pose.pose.position.y,
                                       TAKEOFF_POSE_Z + home_pose.pose.position.z);
    PICKUP_START_WAYPOINT = Eigen::Vector3d(PICKUP_START_POSE_X + home_pose.pose.position.x,
                                            PICKUP_START_POSE_Y + home_pose.pose.position.y,
                                            PICKUP_START_POSE_Z + home_pose.pose.position.z);
    PICKUP_END_WAYPOINT = Eigen::Vector3d(PICKUP_END_POSE_X + home_pose.pose.position.x,
                                          PICKUP_END_POSE_Y + home_pose.pose.position.y,
                                          PICKUP_END_POSE_Z + home_pose.pose.position.z);
    AVOID_START_WAYPOINT = Eigen::Vector3d(AVOID_START_POSE_X + home_pose.pose.position.x,
                                           AVOID_START_POSE_Y + home_pose.pose.position.y,
                                           AVOID_START_POSE_Z + home_pose.pose.position.z);
    AVOID_END_WAYPOINT = Eigen::Vector3d(AVOID_END_POSE_X + home_pose.pose.position.x,
                                         AVOID_END_POSE_Y + home_pose.pose.position.y,
                                         AVOID_END_POSE_Z + home_pose.pose.position.z);
    TRACE_START_WAYPOINT = Eigen::Vector3d(TRACE_START_POSE_X + home_pose.pose.position.x,
                                           TRACE_START_POSE_Y + home_pose.pose.position.y,
                                           TRACE_START_POSE_Z + home_pose.pose.position.z);
    TRACE_END_WAYPOINT = Eigen::Vector3d(TRACE_END_POSE_X + home_pose.pose.position.x,
                                         TRACE_END_POSE_Y + home_pose.pose.position.y,
                                         TRACE_END_POSE_Z + home_pose.pose.position.z);
                                        
}