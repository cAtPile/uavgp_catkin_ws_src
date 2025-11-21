/**
 * @file trace_kit.cpp
 * @brief 跟踪相关
 */
#include "mission_master_pkg/mission_master.h"

/**
 * @brief 跟踪任务开始
 */
void MissionMaster::traceStart()
{
    ROS_INFO("T Start");
    setPoint(trace_start_waypoint_re);

    while (ros::ok())
    {
        ROS_INFO("T Loop");
        setpoint_pub_.publish(temp_pose);
        if (reachCheck(trace_start_waypoint_re))
        {
            ROS_INFO("Arrived at trace Start Point");
            current_mission_state = mission_queue[mission_queue_index];
            mission_queue_index++;
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
    ROS_INFO("T exe");
    traceLoop();

    temp_pose.pose.position.x = current_pose.pose.position.x;
    temp_pose.pose.position.y = current_pose.pose.position.y;
    temp_pose.pose.position.z = trace_end_waypoint_v[2] + home_pose.pose.position.z;

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
 * @brief 跟踪检查
 */
void MissionMaster::traceCheck()
{
    ROS_INFO("T c");

    setPoint(trace_end_waypoint_re);
    while (ros::ok())
    {
        ROS_INFO_THROTTLE(1.0, "TC L");
        setpoint_pub_.publish(temp_pose);
        if (reachCheck(trace_end_waypoint_re))
        {

            ROS_INFO("Arrived at trace End Point");
            current_mission_state = mission_queue[mission_queue_index];
            mission_queue_index++;
            break;
        }

        ros::spinOnce();
        rate_.sleep();
    }
}
void MissionMaster::traceLoop() 
{
    ROS_INFO("TRACE LOOP IN");

    // 步骤1：视觉使能（追踪模式）
    std_msgs::UInt8 traceStart_camCmd_msg;
    traceStart_camCmd_msg.data = 2;
    cam_cmd_pub_.publish(traceStart_camCmd_msg);

    // 步骤2：降落到追踪检测高度
    ROS_INFO("Step 1: Descending to trace detection height (%.2f m)", trace_detect_height);
    setPoint(Eigen::Vector3d(current_pose.pose.position.x, 
                              current_pose.pose.position.y, 
                              trace_detect_height + home_pose.pose.position.z));
    while (ros::ok())
    {
        ROS_INFO_THROTTLE(1.0, "Descending to trace detection height...");
        setpoint_pub_.publish(temp_pose);
        if (reachCheck(Eigen::Vector3d(current_pose.pose.position.x, 
                                        current_pose.pose.position.y, 
                                        trace_detect_height + home_pose.pose.position.z)))
        {
            ROS_INFO("Reached trace detection height");
            break;
        }
        ros::spinOnce();
        rate_.sleep();
    }

    // 步骤3：动态追踪目标并逐渐下降
    ROS_INFO("Step 2: Dynamic tracking with gradual descent");
    
    geometry_msgs::PoseStamped trace_pose;
    trace_pose.header.frame_id = "map";
    trace_pose.pose.orientation = home_orientation;

    double car_x, car_y;
    double rel_cam_x, rel_cam_y;
    double last_seen_time = ros::Time::now().toSec();

    // 追踪主循环
    while (ros::ok())
    {
        ROS_INFO_THROTTLE(1.0, "Tracking target, height: %.2f m", current_pose.pose.position.z);
        
        // 检查目标是否丢失
        if (current_camtrack.car_num == 0)
        {
            double current_time = ros::Time::now().toSec();
            if (current_time - last_seen_time > trace_cam_timeout)
            {
                ROS_WARN("Target lost for too long, exiting trace loop...");
                break;
            }
            ROS_INFO_THROTTLE(1.0, "Waiting for trace target...");
            setpoint_pub_.publish(current_pose);
            ros::spinOnce();
            rate_.sleep();
            continue;
        }
        else
        {
            last_seen_time = ros::Time::now().toSec();
        }

        // 获取目标像素坐标
        car_x = current_camtrack.car_x[0];
        car_y = current_camtrack.car_y[0];

        // 计算相对坐标
        rel_cam_x = car_x - trace_center_x;
        rel_cam_y = car_y - trace_center_y;

        double current_height = current_pose.pose.position.z;
        double cam_loc_rate_h = cam_loc_rate * current_height;

        // 转化为实际坐标（机体坐标系）
        double real_cam_x = rel_cam_x * cam_loc_rate_h;
        double real_cam_y = rel_cam_y * cam_loc_rate_h;
        
        // 从home_orientation提取yaw角
        double siny_cosp = 2.0 * (home_orientation.w * home_orientation.z + home_orientation.x * home_orientation.y);
        double cosy_cosp = 1.0 - 2.0 * (home_orientation.y * home_orientation.y + home_orientation.z * home_orientation.z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);
        
        // 将机体坐标系的偏差转换为世界坐标系
        double real_cam_x_world = std::cos(yaw) * real_cam_x - std::sin(yaw) * real_cam_y;
        double real_cam_y_world = std::sin(yaw) * real_cam_x + std::cos(yaw) * real_cam_y;

        // 计算目标高度，逐步下降
        double target_height = current_height - step_size_trace;

        // 限制最低高度为投放高度
        if (target_height < trace_release_height + home_pose.pose.position.z)
        {
            target_height = trace_release_height + home_pose.pose.position.z;
        }

        // 设置追踪目标点（世界坐标系）
        trace_pose.pose.position.x = current_pose.pose.position.x - real_cam_x_world;
        trace_pose.pose.position.y = current_pose.pose.position.y - real_cam_y_world;
        trace_pose.pose.position.z = target_height;

        ROS_INFO("Target: x=%.2f, y=%.2f, z=%.2f, pixel_err=(%.1f, %.1f)", 
                 trace_pose.pose.position.x, trace_pose.pose.position.y, trace_pose.pose.position.z,
                 rel_cam_x, rel_cam_y);

        // 发送飞行指令
        setpoint_pub_.publish(trace_pose);

        // 检查是否达到投放条件：1.高度达到投放高度  2.目标在容忍范围内
        if (current_pose.pose.position.z <= (trace_release_height + home_pose.pose.position.z + 0.05) &&
            real_cam_x * real_cam_x + real_cam_y * real_cam_y <= tolerance_pix * tolerance_pix)
        {
            ROS_INFO("Release conditions met: height=%.2f, pixel_error=%.2f", 
                     current_pose.pose.position.z, 
                     std::sqrt(real_cam_x * real_cam_x + real_cam_y * real_cam_y));
            break; // 满足条件，退出追踪循环
        }

        ros::spinOnce();
        rate_.sleep();
    }

    // 步骤4：调用投放服务
    ROS_INFO("Step 3: Calling release service");
    if (gripRelease())
    {
        ROS_INFO("Release service returned success");
    }
    else
    {
        ROS_WARN("Release service returned failure");
    }

    // 步骤5：升高到追踪完成高度
    ROS_INFO("Step 4: Ascending to trace success height (%.2f m)", trace_success_height);
    setPoint(Eigen::Vector3d(current_pose.pose.position.x, 
                              current_pose.pose.position.y, 
                              trace_success_height + home_pose.pose.position.z));
    while (ros::ok())
    {
        ROS_INFO_THROTTLE(1.0, "Ascending to trace success height...");
        setpoint_pub_.publish(temp_pose);
        if (reachCheck(Eigen::Vector3d(current_pose.pose.position.x, 
                                        current_pose.pose.position.y, 
                                        trace_success_height + home_pose.pose.position.z)))
        {
            ROS_INFO("Reached trace success height");
            break;
        }
        ros::spinOnce();
        rate_.sleep();
    }

    // 视觉休息
    std_msgs::UInt8 traceEnd_camCmd_msg;
    traceEnd_camCmd_msg.data = 0;
    cam_cmd_pub_.publish(traceEnd_camCmd_msg);
    
    ROS_INFO("TRACE LOOP COMPLETED");
}

bool MissionMaster::gripRelease()
{

    // 等待 Action 服务器启动
    if (!gripper_ac_.waitForServer(ros::Duration(5.0))) // 等待最多 5 秒
    {
        ROS_ERROR("Unable to connect to gripper action server!");
        return false; // 连接不到服务器，返回失败
    }

    // 创建目标消息并设置命令为释放
    mission_master_pkg::GripGoal goal;
    goal.command = mission_master_pkg::GripGoal::RELEASE; // 命令为释放

    // 发送目标
    gripper_ac_.sendGoal(goal);

    // 设置超时时间
    ros::Time start_time = ros::Time::now();
    double timeout_duration = 10.0; // 等待超时时间，单位：秒

    // 等待释放完成的过程中，允许处理回调函数
    while (ros::ok())
    {
        // 检查是否超时
        if ((ros::Time::now() - start_time).toSec() > timeout_duration)
        {
            ROS_ERROR("Gripper release action did not finish before the timeout.");
            gripper_ac_.cancelGoal(); // 超时取消目标
            return false;             // 超时未完成释放，返回失败
        }

        // 检查是否完成释放任务
        if (gripper_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            // 获取结果
            const mission_master_pkg::GripResult::ConstPtr &result = gripper_ac_.getResult();

            // 根据 cmd_success 判断释放是否成功
            if (result->cmd_success)
            {
                ROS_INFO("Gripper successfully released the object!");
                return true; // 释放成功，返回 true
            }
            else
            {
                ROS_WARN("Gripper failed to release the object!");
                return false; // 释放失败，返回 false
            }
        }

        // 处理回调函数
        ros::spinOnce(); // 调用一次回调函数

        // 控制循环的频率
        ros::Rate(10).sleep(); // 例如设置频率为 10Hz，控制循环速度
    }

    return false; // 如果退出了循环，表示某种错误发生
}
