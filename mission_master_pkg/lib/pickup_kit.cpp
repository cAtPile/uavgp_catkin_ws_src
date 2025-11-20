#include "mission_master_pkg/mission_master.h"
/**
 * @brief 抓取开始点
 */
void MissionMaster::pickupStart()
{
    ROS_DEBUG("P start");

    setPoint(pickup_start_waypoint_re);

    while (ros::ok())
    {
        ROS_DEBUG("P loop");

        setpoint_pub_.publish(temp_pose);
        if (reachCheck(pickup_start_waypoint_re))
        {
            ROS_INFO("Arrived at Pickup Start Point");
            current_mission_state = mission_queue[mission_queue_index];
            mission_queue_index++;
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
    ROS_DEBUG("EXE PICKUP");

    pickLoop();

    temp_pose.pose.position.x = current_pose.pose.position.x;
    temp_pose.pose.position.y = current_pose.pose.position.y;
    temp_pose.pose.position.z = pickup_end_waypoint_v[2] + home_pose.pose.position.z;

    while (ros::ok())
    {

        setpoint_pub_.publish(temp_pose);
        if (reachCheck(Eigen::Vector3d(temp_pose.pose.position.x, temp_pose.pose.position.y, temp_pose.pose.position.z)))
        {
            current_mission_state = mission_queue[mission_queue_index];
            mission_queue_index++;
            break;
        }
        ros::spinOnce();
        rate_.sleep();
    }
}

/**
 * @brief 抓取任务结束检查
 */
void MissionMaster::pickupCheck()
{
    ROS_INFO("EXE PICK END");

    setPoint(pickup_end_waypoint_re);
    while (ros::ok())
    {
        ROS_INFO_THROTTLE(1.0, " EXE PICKUP END LOOP ");

        setpoint_pub_.publish(temp_pose);
        if (reachCheck(pickup_end_waypoint_re))
        {
            ROS_INFO("EXE PICKUP END CHECK ");

            current_mission_state = mission_queue[mission_queue_index];
            mission_queue_index++;
            break;
        }
        ros::spinOnce();
        rate_.sleep();
    }
}

void MissionMaster::pickLoop()
{
    ROS_INFO("PICK LOOP IN");

    // 视觉使能（抓）
    std_msgs::UInt8 pickupStart_camCmd_msg;
    pickupStart_camCmd_msg.data = 1;
    cam_cmd_pub_.publish(pickupStart_camCmd_msg);

    // 步骤1：降落到检测高度
    ROS_INFO("Step 1: Descending to detection height (%.2f m)", pickup_detect_height);
    setPoint(Eigen::Vector3d(current_pose.pose.position.x, 
                              current_pose.pose.position.y, 
                              pickup_detect_height + home_pose.pose.position.z));
    while (ros::ok())
    {
        ROS_INFO_THROTTLE(1.0, "Descending to detection height...");
        setpoint_pub_.publish(temp_pose);
        if (reachCheck(Eigen::Vector3d(current_pose.pose.position.x, 
                                        current_pose.pose.position.y, 
                                        pickup_detect_height + home_pose.pose.position.z)))
        {
            ROS_INFO("Reached detection height");
            break;
        }
        ros::spinOnce();
        rate_.sleep();
    }

    // 步骤2：视觉伺服对准
    ROS_INFO("Step 2: Vision servoing for alignment");
    visionLoop(pickup_center_x, pickup_center_y);
    ROS_INFO("Vision alignment completed");

    // 步骤3：降落到抓取高度
    ROS_INFO("Step 3: Descending to grip height (%.2f m)", pickup_grip_height);
    setPoint(Eigen::Vector3d(current_pose.pose.position.x, 
                              current_pose.pose.position.y, 
                              pickup_grip_height + home_pose.pose.position.z));
    while (ros::ok())
    {
        ROS_INFO_THROTTLE(1.0, "Descending to grip height...");
        setpoint_pub_.publish(temp_pose);
        if (reachCheck(Eigen::Vector3d(current_pose.pose.position.x, 
                                        current_pose.pose.position.y, 
                                        pickup_grip_height + home_pose.pose.position.z)))
        {
            ROS_INFO("Reached grip height");
            break;
        }
        ros::spinOnce();
        rate_.sleep();
    }

    // 步骤4：调用抓取服务
    ROS_INFO("Step 4: Calling grip service");
    if (gripPick())
    {
        ROS_INFO("Grip service returned success");
    }
    else
    {
        ROS_WARN("Grip service returned failure");
    }

    // 步骤5：升高到抓取成功高度
    ROS_INFO("Step 5: Ascending to success height (%.2f m)", pickup_success_height);
    setPoint(Eigen::Vector3d(current_pose.pose.position.x, 
                              current_pose.pose.position.y, 
                              pickup_success_height + home_pose.pose.position.z));
    while (ros::ok())
    {
        ROS_INFO_THROTTLE(1.0, "Ascending to success height...");
        setpoint_pub_.publish(temp_pose);
        if (reachCheck(Eigen::Vector3d(current_pose.pose.position.x, 
                                        current_pose.pose.position.y, 
                                        pickup_success_height + home_pose.pose.position.z)))
        {
            ROS_INFO("Reached success height");
            break;
        }
        ros::spinOnce();
        rate_.sleep();
    }

    // 视觉休息
    std_msgs::UInt8 pickupEnd_camCmd_msg;
    pickupEnd_camCmd_msg.data = 0;
    cam_cmd_pub_.publish(pickupEnd_camCmd_msg);

    ROS_INFO("PICK LOOP COMPLETED");
}

bool MissionMaster::gripPick()
{
    // 创建抓取的 Action 客户端
    actionlib::SimpleActionClient<mission_master_pkg::GripAction> ac("gripper_action", true);

    // 等待 Action 服务器启动
    if (!ac.waitForServer(ros::Duration(5.0))) // 等待最多 5 秒
    {
        ROS_ERROR("Unable to connect to gripper action server!");
        return false; // 连接不到服务器，返回失败
    }

    // 创建目标消息并设置命令为抓取
    mission_master_pkg::GripGoal goal;
    goal.command = mission_master_pkg::GripGoal::GRIP; // 命令为抓取

    // 发送目标
    ac.sendGoal(goal);

    // 等待结果，最多等待 10 秒
    bool finished_before_timeout = ac.waitForResult(ros::Duration(10.0));

    // 获取结果
    if (finished_before_timeout)
    {
        // 获取结果
        const mission_master_pkg::GripResult::ConstPtr &result = ac.getResult();

        // 根据 cmd_success 判断抓取是否成功
        if (result->cmd_success)
        {
            ROS_INFO("Gripper successfully picked up the object!");
            return true; // 抓取成功，返回 true
        }
        else
        {
            ROS_WARN("Gripper failed to pick up the object!");
            return false; // 抓取失败，返回 false
        }
    }
    else
    {
        ROS_ERROR("Gripper action did not finish before the timeout.");
        return false; // 超时未完成抓取，返回失败
    }
}