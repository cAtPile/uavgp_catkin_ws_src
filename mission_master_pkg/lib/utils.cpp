#include "mission_master_pkg/mission_master.h"
#include <cmath>

/**
 * @brief 机体坐标系转世界坐标系
 * @param point_body 机体坐标系下的点 (x_body, y_body, z_body)
 * @return 世界坐标系下的点 (x_world, y_world, z_world)
 * @details 将相对于无人机机体的坐标转换为世界坐标系（ENU）
 *          机体坐标系：x前 y左 z上
 *          世界坐标系：x东 y北 z上
 */
Eigen::Vector3d MissionMaster::bodyFrameToWorld(const Eigen::Vector3d &point_body)
{
    // 从home_orientation提取yaw角
    double siny_cosp = 2.0 * (home_orientation.w * home_orientation.z + home_orientation.x * home_orientation.y);
    double cosy_cosp = 1.0 - 2.0 * (home_orientation.y * home_orientation.y + home_orientation.z * home_orientation.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    
    // 2D旋转矩阵（绕z轴旋转yaw角）
    // x_world = cos(yaw) * x_body - sin(yaw) * y_body
    // y_world = sin(yaw) * x_body + cos(yaw) * y_body
    // z_world = z_body
    
    Eigen::Vector3d point_world;
    point_world(0) = std::cos(yaw) * point_body(0) - std::sin(yaw) * point_body(1) + current_pose.pose.position.x;
    point_world(1) = std::sin(yaw) * point_body(0) + std::cos(yaw) * point_body(1) + current_pose.pose.position.y;
    point_world(2) = point_body(2) + current_pose.pose.position.z;
    
    return point_world;
}

/**
 * @brief 定点飞行（机体坐标系）
 * @param set_point_body 相对于机体坐标系的目标点
 * @details 将机体坐标系的目标点转换为世界坐标系后发送
 */
void MissionMaster::setPointBodyFrame(const Eigen::Vector3d &set_point_body)
{
    // 转换到世界坐标系
    Eigen::Vector3d set_point_world = bodyFrameToWorld(set_point_body);
    
    // 使用世界坐标系的setPoint函数
    setPoint(set_point_world);
}

/**
 * @brief 定点（使用home位置的偏航角）
 */

void MissionMaster::setPoint(const Eigen::Vector3d &set_point)
{
    temp_pose.header.frame_id = "map";
    temp_pose.header.stamp = ros::Time::now();
    temp_pose.pose.position.x = set_point(0);
    temp_pose.pose.position.y = set_point(1);
    temp_pose.pose.position.z = set_point(2);
    // 使用home位置的偏航角（四元数形式）
    temp_pose.pose.orientation = home_orientation;
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
    // 使用home位置的偏航角（四元数形式）
    vision_pose.pose.orientation = home_orientation;

    // 临时参数
    double field_view_x = 720; // 视场
    double field_view_y = 1280; //
    double drone_cam_rate = 1; // 控制量
    double cam_timeout = 5.0;  // 例子超时值，单位：秒

    // 局部参数
    double delta_ball_x_pix, delta_ball_y_pix;
    double delta_ball_x_pix_correct, delta_ball_y_pix_correct;
    double ball_x_rate, ball_y_rate;
    double drone_step_x, drone_step_y;

    // 超时设置
    double last_seen_time = ros::Time::now().toSec();
    double start_cam_time = ros::Time::now().toSec();

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
            start_cam_time = ros::Time::now().toSec();
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

        //超时等待
        if(ros::Time::now().toSec()-start_cam_time>10){
            ROS_INFO("TIMEOUT");
            break;
        }

        // 退出条件
        if (current_camtrack.ball_x * current_camtrack.ball_x + current_camtrack.ball_y * current_camtrack.ball_y <= tolerance_pix * tolerance_pix)
        {
            ROS_INFO("Vision in tolerance");
            break;
        }
        ROS_INFO("camx = %.2f ;  camy = %.2f ", current_camtrack.ball_x, current_camtrack.ball_y);

        // 相对化
        delta_ball_x_pix = current_camtrack.ball_x - aim_x;
        delta_ball_y_pix = current_camtrack.ball_y - aim_y;
        ROS_INFO("dx = %.2f ;  dy = %.2f ", delta_ball_x_pix, delta_ball_y_pix);

        // 修正
        delta_ball_x_pix_correct =  current_pose.pose.position.z * delta_ball_x_pix;
        delta_ball_y_pix_correct =  current_pose.pose.position.z * delta_ball_y_pix;
        ROS_INFO("cx = %.2f ;  cy = %.2f ", delta_ball_x_pix_correct, delta_ball_y_pix_correct);

        // 比例化
        ball_x_rate = delta_ball_x_pix_correct / field_view_x;
        ball_y_rate = delta_ball_y_pix_correct / field_view_y;

        ROS_INFO("rx = %.2f ;  ry = %.2f ", ball_x_rate, ball_y_rate);

        // 计算控制量
        drone_step_x = drone_cam_rate * ball_x_rate;
        drone_step_y = drone_cam_rate * ball_y_rate;

        ROS_INFO("vx = %.2f ;  vy = %.2f ", drone_step_x, drone_step_y);

        // 定点量设置
        vision_pose.pose.position.x = current_pose.pose.position.x - drone_step_x;
        vision_pose.pose.position.y = current_pose.pose.position.y - drone_step_y;
        vision_pose.pose.position.z = current_pose.pose.position.z; // 临时

        ROS_INFO("x = %.2f ;  y = %.2f ; z = %.2f ", vision_pose.pose.position.x, vision_pose.pose.position.y, vision_pose.pose.position.z);

        // 发布定点
        setpoint_pub_.publish(vision_pose);

        ros::spinOnce();
        rate_.sleep();
    }
}
