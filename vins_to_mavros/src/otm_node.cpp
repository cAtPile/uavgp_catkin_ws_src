#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <cmath>

// 定义坐标和四元数
Eigen::Vector3d p_lidar_body, p_enu;
Eigen::Quaterniond q_mav;
Eigen::Quaterniond q_px4_odom;

// 从四元数计算偏航角
double fromQuaternion2yaw(Eigen::Quaterniond q) {
    double yaw = atan2(2 * (q.x() * q.y() + q.w() * q.z()), q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
    return yaw;
}

// VINS回调函数，用于接收VINS数据
void vins_callback(const nav_msgs::Odometry::ConstPtr &msg) {
    p_lidar_body = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    q_mav = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
}

// PX4本地位置回调，用于处理PX4的姿态数据
void px4_odom_callback(const nav_msgs::Odometry::ConstPtr &msg) {
    // 直接使用接收到的四元数数据
    q_px4_odom = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
}

// 主函数
int main(int argc, char **argv) {
    ros::init(argc, argv, "vins_to_mavros");
    ros::NodeHandle nh("~");

    // 订阅VINS和PX4的本地位置
    ros::Subscriber slam_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 100, vins_callback);
    ros::Subscriber px4_odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 5, px4_odom_callback);

    // 发布视觉位置
    ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

    ros::Rate rate(20.0);  // 设置发布频率为20Hz

    ros::Time last_request = ros::Time::now();
    float init_yaw = 0.0;
    bool init_flag = 0;
    Eigen::Quaterniond init_q;

    while (ros::ok()) {
        // 计算初始化的偏航角（如果需要）
        if (!init_flag) {
            init_yaw = fromQuaternion2yaw(q_px4_odom);  // 从PX4的四元数计算偏航角
            init_flag = 1;
            // 设置初始旋转矩阵（仅对Z轴进行初始化）
            init_q = Eigen::AngleAxisd(init_yaw, Eigen::Vector3d::UnitZ())
                     * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
                     * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
        }

        // 在初始化完成后发布视觉位置信息
        if (init_flag) {
            geometry_msgs::PoseStamped vision;

            // 将lidar_body系转换为ENU系
            p_enu = init_q * p_lidar_body;

            // 填充位置数据
            vision.pose.position.x = p_enu[0];
            vision.pose.position.y = p_enu[1];
            vision.pose.position.z = p_enu[2];

            // 填充姿态数据（使用VINS的四元数）
            // 对VINS的四元数进行方向修正
            vision.pose.orientation.x = -q_mav.x();
            vision.pose.orientation.y = -q_mav.y();
            vision.pose.orientation.z = -q_mav.z();
            vision.pose.orientation.w = -q_mav.w();

            // 设置坐标系为 "map"
            vision.header.frame_id = "map";
            vision.header.stamp = ros::Time::now();  // 设置时间戳
            vision_pub.publish(vision);

            // 打印调试信息
            ROS_INFO("\nposition in enu:\n   x: %.18f\n   y: %.18f\n   z: %.18f\norientation of lidar:\n   x: %.18f\n   y: %.18f\n   z: %.18f\n   w: %.18f",
                     p_enu[0], p_enu[1], p_enu[2], q_mav.x(), q_mav.y(), q_mav.z(), q_mav.w());
        }

        ros::spinOnce();  // 处理回调函数
        rate.sleep();     // 睡眠，保持频率
    }

    return 0;
}
