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
    //PIDController pid_x(xkp, xki, xkd,xomn,xoma,ximn,xima);  // 对应 x 方向的 PID 参数
    //PIDController pid_y(ykp, yki, ykd,yomn,yoma,yimn,yima);  // 对应 x 方向的 PID 参数

    PIDController pid_x(0.5, 0.3, 0.8,-320,320,-320,320);  // 对应 x 方向的 PID 参数
PIDController pid_y(0.5, 0.3, 0.8,-320,320,-320,320);  // 对应 y 方向的 PID 参数
    ROS_INFO("PICK LOOP IN");

    // 视觉使能（抓）
    std_msgs::UInt8 pickupStart_camCmd_msg;
    pickupStart_camCmd_msg.data = 1;
    cam_cmd_pub_.publish(pickupStart_camCmd_msg);

    // pick_pose 初始化
    geometry_msgs::PoseStamped pick_pose;
    pick_pose.header.frame_id = "map";
    pick_pose.pose.orientation.x = 0;
    pick_pose.pose.orientation.y = 0;
    pick_pose.pose.orientation.z = 0;
    pick_pose.pose.orientation.w = 1;

    double ball_x, ball_y;
    double rel_cam_x, rel_cam_y;

    // 超时设置
    double last_seen_time = ros::Time::now().toSec();

    // 抓取主循环
    while (ros::ok())
    {
        ROS_INFO_THROTTLE(1.0, "PICK LOOP PULSE");
        
        // 如果目标没有被检测到，进入等待状态
        if (current_camtrack.ball_num == 0)
        {
            double current_time = ros::Time::now().toSec();
            // 如果超时，则跳出循环
            if (current_time - last_seen_time > pickup_cam_timeout)
            {
                ROS_WARN("Target lost for too long, exiting pick loop...");
                break;
            }
            ROS_INFO_THROTTLE(1.0, "Waiting for target...");
            setpoint_pub_.publish(current_pose);
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

        double current_height = current_pose.pose.position.z;
        double cam_loc_rate_h = current_height * cam_loc_rate;

        // 计算相对坐标，减去图像中心
        rel_cam_x = ball_x - pickup_center_x;
        rel_cam_y = ball_y - pickup_center_y;

        // 使用 PID 控制器来计算 delta_x 和 delta_y
        double pid_output_x = pid_x.compute(rel_cam_x, 0.1);  // 使用 PID 控制器调整 x 方向
        double pid_output_y = pid_y.compute(rel_cam_y, 0.1);  // 使用 PID 控制器调整 y 方向

        // 转化为实际坐标
        double delta_x = pid_output_x * cam_loc_rate_h;
        double delta_y = pid_output_y * cam_loc_rate_h;

        // 计算当前高度，逐步下降
        double target_height = current_height - pickup_step_size; // 目标高度逐步降低

        // 如果高度已经降到目标高度以下，就将高度设置为目标高度
        if (target_height < pickup_aim_high)
        {
            target_height = pickup_aim_high;
        }

        // 如果目标在容忍范围内，则进行抓取
        if (rel_cam_x * rel_cam_x + rel_cam_y * rel_cam_y <= tolerace_pix * tolerace_pix)
        {
            // 当前飞行路径调整到目标位置
            pick_pose.pose.position.x = current_pose.pose.position.x - delta_x;
            pick_pose.pose.position.y = current_pose.pose.position.y - delta_y;
            pick_pose.pose.position.z = target_height; // 高度逐步调整

            ROS_INFO("x:%0.2f,y:%0.2f,z:%0.2f", pick_pose.pose.position.x, pick_pose.pose.position.y, pick_pose.pose.position.z);

            // 发送飞行指令
            setpoint_pub_.publish(pick_pose);

            // 检查是否到达目标位置并准备抓取
            if (current_pose.pose.position.z <= pickup_aim_high)
            {
                if (1 /*gripPick()*/) // 调用抓取函数
                {
                    ROS_INFO("PICK LOOP out");
                    break; // 跳出抓取循环
                }
            }
        }
        else
        {
            // 如果目标物体不在容忍范围内，继续调整位置
            pick_pose.pose.position.x = current_pose.pose.position.x - delta_x;
            pick_pose.pose.position.y = current_pose.pose.position.y - delta_y;
            pick_pose.pose.position.z = current_pose.pose.position.z; // 高度保持逐步下降

            ROS_INFO("x:%0.2f,y:%0.2f,z:%0.2f", pick_pose.pose.position.x, pick_pose.pose.position.y, pick_pose.pose.position.z);

            // 发送调整后的飞行指令
            setpoint_pub_.publish(pick_pose);
        }

        ros::spinOnce();
        rate_.sleep();
    }

    // 视觉休息
    std_msgs::UInt8 pickupEnd_camCmd_msg;
    pickupEnd_camCmd_msg.data = 0;
    cam_cmd_pub_.publish(pickupEnd_camCmd_msg);
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