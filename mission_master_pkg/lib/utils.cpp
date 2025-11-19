#include "mission_master_pkg/mission_master.h"
/**
 * @brief 定点
 */

void MissionMaster::setPoint(const Eigen::Vector3d &set_point)
{

    temp_pose.header.frame_id = "map";
    temp_pose.header.stamp = ros::Time::now();
    temp_pose.pose.position.x = set_point(0);
    temp_pose.pose.position.y = set_point(1);
    temp_pose.pose.position.z = set_point(2);
    temp_pose.pose.orientation.x = 0.0;
    temp_pose.pose.orientation.y = 0.0;
    temp_pose.pose.orientation.z = 0.0;
    temp_pose.pose.orientation.w = 1.0;
}

/**
 * @brief 到达检查
 */

bool MissionMaster::reachCheck(const Eigen::Vector3d &check_point)
{
    double dx = current_pose.pose.position.x - check_point(0);
    double dy = current_pose.pose.position.y - check_point(1);
    double dz = current_pose.pose.position.z - check_point(2);
    double distance = sqrt(dx * dx + dy * dy + dz * dz);
    return distance < TOLERANCE_WAYPOINT;
}

/**
 * @brief 视伺服
 * @param aim_x 目标x
 * @param aim_y 目标y
 */
void MissionMaster::visionLoop(double aim_x, double aim_y)
{
    // 日志
    ROS_INFO("Vision Loop Enter");

    // pick_pose 初始化
    geometry_msgs::PoseStamped vision_pose;
    vision_pose.header.frame_id = "map";
    vision_pose.pose.orientation.x = 0;
    vision_pose.pose.orientation.y = 0;
    vision_pose.pose.orientation.z = 0;
    vision_pose.pose.orientation.w = 1;

    // 临时参数
    double field_view_x = 640;   // 视场
    double field_view_y = 480;   //
    double drone_cam_rate = 0.1; //控制量
    double cam_timeout = 5.0;    // 例子超时值，单位：秒

    // 局部参数
    double delta_ball_x_pix, delta_ball_y_pix;
    double delta_ball_x_pix_correct, delta_ball_y_pix_correct;
    double ball_x_rate, ball_y_rate;
    double drone_step_x, drone_step_y;

    // 超时设置
    double last_seen_time = ros::Time::now().toSec();

    // 主循环
    while (ros::ok())
    {
        ROS_INFO("Vision Loop Pluse");

        // 等待状态
        if (current_camtrack.ball_num == 0)
        {
            double current_time = ros::Time::now().toSec();
            // 如果超时，则跳出循环
            if (current_time - last_seen_time > cam_timeout)
            {
                ROS_WARN("Target lost for too long, exiting vision loop...");
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

        // 退出条件
        if (current_camtrack.ball_x * current_camtrack.ball_x + current_camtrack.ball_y * current_camtrack.ball_y <= tolerance_pix * tolerance_pix)
        {
            ROS_INFO("Vision in tolerance");
            break;
        }

        // 相对化
        delta_ball_x_pix = current_camtrack.ball_x - aim_x;
        delta_ball_y_pix = current_camtrack.ball_y - aim_y;

        // 修正
        delta_ball_x_pix_correct = cam_loc_rate * current_pose.pose.position.z * delta_ball_x_pix;
        delta_ball_y_pix_correct = cam_loc_rate * current_pose.pose.position.z * delta_ball_y_pix;

        // 比例化
        ball_x_rate = delta_ball_x_pix_correct / field_view_x;
        ball_y_rate = delta_ball_y_pix_correct / field_view_y;

        // 计算控制量
        drone_step_x = drone_cam_rate * ball_x_rate;
        drone_step_y = drone_cam_rate * ball_y_rate;

        // 定点量设置
        vision_pose.pose.position.x = current_x - drone_step_x;
        vision_pose.pose.position.y = current_y - drone_step_y;
        vision_pose.pose.position.z = current_pose.position.z; // 临时

        ROS_INFO("x = vision_pose.pose.position.x ;  y = vision_pose.pose.position.y ; z = vision_pose.pose.position.z ")

        // 发布定点
        setpoint_pub_.publish(vision_pose);

        ros::spinOnce();
        rate_.sleep();
    }
}
