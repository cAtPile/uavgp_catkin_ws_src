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
    setPoint(TRACE_START_WAYPOINT);

    while (ros::ok())
    {
        ROS_INFO("T Loop");
        setpoint_pub_.publish(temp_pose);
        if (reachCheck(TRACE_START_WAYPOINT))
        {
            ROS_INFO("Arrived at trace Start Point");
            current_mission_state = EXECUTE_TRACE_STATE;
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
    temp_pose.pose.position.z = 5.0 + home_pose.pose.position.z;

    while (ros::ok())
    {

        setpoint_pub_.publish(temp_pose);
        if (reachCheck(Eigen::Vector3d(temp_pose.pose.position.x, temp_pose.pose.position.y, temp_pose.pose.position.z)))
        {
            current_mission_state = SUCCEED_TRACE_STATE;
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

    setPoint(TRACE_END_WAYPOINT);
    while (ros::ok())
    {
        ROS_INFO("TC L");
        setpoint_pub_.publish(temp_pose);
        if (reachCheck(TRACE_END_WAYPOINT))
        {

            ROS_INFO("Arrived at trace End Point");
            current_mission_state = START_LAND_STATE;
            break;
        }

        ros::spinOnce();
        rate_.sleep();
    }
}
void MissionMaster::traceLoop()
{
    ROS_INFO("TR LOOP IN");

    // pick_pose 初始化
    geometry_msgs::PoseStamped trace_pose;
    trace_pose.header.frame_id = "map";
    trace_pose.pose.orientation.x = 0;
    trace_pose.pose.orientation.y = 0;
    trace_pose.pose.orientation.z = 0;
    trace_pose.pose.orientation.w = 1;

    double car_x, car_y;
    double rel_cam_x, rel_cam_y;

    // 超时设置
    double timeout_duration = 1.0; // 等待超时
    double last_seen_time = ros::Time::now().toSec();

    // 抓取主循环
    while (ros::ok())
    {
        ROS_INFO("TR LOOP pose");
        // 如果目标没有被检测到，进入等待状态
        if (current_camtrack.car_num == 0)
        {
            double current_time = ros::Time::now().toSec();
            // 如果超时，则跳出循环
            if (current_time - last_seen_time > timeout_duration)
            {
                ROS_WARN("Target lost for too long, exiting TR loop...");
                break;
            }
            // 如果目标丢失，更新最后一次看到目标的时间
            last_seen_time = current_time;
            ROS_INFO("Waiting for target...TR");
            ros::spinOnce();
            rate_.sleep();
            continue;
        }
        else
        {
            // 目标重新检测到，重置等待计时器
            last_seen_time = ros::Time::now().toSec();
        }

        // 获取像素坐标
        car_x = current_camtrack.car_x[0];
        car_y = current_camtrack.car_y[0];

        // 计算相对坐标，减去图像中心
        rel_cam_x = car_x - trace_center_x;
        rel_cam_y = car_y - trace_center_y;

        // 转化为实际坐标
        rel_cam_x = rel_cam_x * cam_loc_rate;
        rel_cam_y = rel_cam_y * cam_loc_rate;

        // 计算当前高度，逐步下降
        double current_height = current_pose.pose.position.z;
        double target_height = current_height - step_size_trace; // 目标高度逐步降低

        // 如果高度已经降到目标高度以下，就将高度设置为目标高度
        if (target_height < aim_high_trace)
        {
            target_height = aim_high_trace;
        }

        // 如果目标在容忍范围内，则进行抓取
        if (rel_cam_x * rel_cam_x + rel_cam_y * rel_cam_y <= tolerace_pix * tolerace_pix)
        {
            // 当前飞行路径调整到目标位置
            trace_pose.pose.position.x = current_pose.pose.position.x - rel_cam_x;
            trace_pose.pose.position.y = current_pose.pose.position.y - rel_cam_y;
            trace_pose.pose.position.z = target_height; // 高度逐步调整

            // 发送飞行指令
            setpoint_pub_.publish(trace_pose);

            // 检查是否到达目标位置并准备抓取
            if (current_pose.pose.position.z <= aim_high_trace && rel_cam_x * rel_cam_x + rel_cam_y * rel_cam_y <= tolerace_pix)
            {
                if (gripRelease()) // 调用释放函数
                {
                    ROS_INFO("TR LOOP out");
                    break; // 跳出跟踪循环
                }
            }
        }
        else
        {
            // 如果目标物体不在容忍范围内，继续调整位置
            trace_pose.pose.position.x = current_pose.pose.position.x - rel_cam_x;
            trace_pose.pose.position.y = current_pose.pose.position.y - rel_cam_y;
            trace_pose.pose.position.z = target_height; // 高度保持逐步下降

            // 发送调整后的飞行指令
            setpoint_pub_.publish(trace_pose);
        }

        ros::spinOnce();
        rate_.sleep();
    }
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

        // 处理回调函数（在等待过程中继续处理ROS消息）
        ros::spinOnce(); // 调用一次回调函数

        // 控制循环的频率
        ros::Rate(10).sleep(); // 例如设置频率为 10Hz，控制循环速度
    }

    return false; // 如果退出了循环，表示某种错误发生
}
