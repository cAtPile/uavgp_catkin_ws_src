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

    current_mission_state = SUCCEED_TRACE_STATE;
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

            ROS_INFO("Arrived at trace End Point")
            current_mission_state = START_LAND_STATE;
            break;
        }

        ros::spinOnce();
        rate_.sleep();
    }
}
void MissionMaster::pickLoop()
{
    ROS_INFO("PICK LOOP IN");

    // pick_pose 初始化
    geometry_msgs::PoseStamped pick_pose;
    pick_pose.header.frame_id = "map";
    pick_pose.pose.orientation.x = 0;
    pick_pose.pose.orientation.y = 0;
    pick_pose.pose.orientation.z = 0;
    pick_pose.pose.orientation.w = 1;

    double ball_x, ball_y;
    double track_center_x, track_center_y;
    double rel_cam_x, rel_cam_y;
    double cam_loc_rate;
    double aim_high;
    double tolerace_pix;
    double step_size = 0.1; // 每次降落的步长

    // 超时设置
    double timeout_duration = 10.0; // 等待超时
    double last_seen_time = ros::Time::now().toSec();

    // 抓取主循环
    while (ros::ok())
    {
        ROS_INFO("PICK LOOP pose");
        // 如果目标没有被检测到，进入等待状态
        if (current_camtrack.ball_num == 0)
        {
            double current_time = ros::Time::now().toSec();
            // 如果超时，则跳出循环
            if (current_time - last_seen_time > timeout_duration)
            {
                ROS_WARN("Target lost for too long, exiting pick loop...");
                break;
            }
            // 如果目标丢失，更新最后一次看到目标的时间
            last_seen_time = current_time;
            ROS_INFO("Waiting for target...");
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
        ball_x = current_camtrack.ball_x;
        ball_y = current_camtrack.ball_y;

        // 计算相对坐标，减去图像中心
        rel_cam_x = ball_x - track_center_x;
        rel_cam_y = ball_y - track_center_y;

        // 转化为实际坐标
        rel_cam_x = rel_cam_x * cam_loc_rate;
        rel_cam_y = rel_cam_y * cam_loc_rate;

        // 计算当前高度，逐步下降
        double current_height = current_pose.pose.position.z;
        double target_height = current_height - step_size; // 目标高度逐步降低

        // 如果高度已经降到目标高度以下，就将高度设置为目标高度
        if (target_height < aim_high)
        {
            target_height = aim_high;
        }

        // 如果目标在容忍范围内，则进行抓取
        if (rel_cam_x * rel_cam_x + rel_cam_y * rel_cam_y <= tolerace_pix * tolerace_pix)
        {
            // 当前飞行路径调整到目标位置
            pick_pose.pose.position.x = current_pose.pose.position.x - rel_cam_x;
            pick_pose.pose.position.y = current_pose.pose.position.y - rel_cam_y;
            pick_pose.pose.position.z = target_height; // 高度逐步调整

            // 发送飞行指令
            setpoint_pub_.publish(pick_pose);

            // 检查是否到达目标位置并准备抓取
            if (current_pose.pose.position.z <= aim_high && rel_cam_x * rel_cam_x + rel_cam_y * rel_cam_y <= tolerace_pix)
            {
                if (gripPick()) // 调用抓取函数
                {
                    ROS_INFO("PICK LOOP out");
                    break; // 跳出抓取循环
                }
            }
        }
        else
        {
            // 如果目标物体不在容忍范围内，继续调整位置
            pick_pose.pose.position.x = current_pose.pose.position.x - rel_cam_x;
            pick_pose.pose.position.y = current_pose.pose.position.y - rel_cam_y;
            pick_pose.pose.position.z = target_height; // 高度保持逐步下降

            // 发送调整后的飞行指令
            setpoint_pub_.publish(pick_pose);
        }

        ros::spinOnce();
        rate_.sleep();
    }
}
