#include "apoc_pkg/apoc.h"

bool apoc::flytoAbsolute(float fly_ab_x, float fly_ab_y, float fly_ab_z, float fly_ab_yaw) {

    // 初始化目标位置消息
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "map"; 
    target_pose.pose.position.x = fly_ab_x;
    target_pose.pose.position.y = fly_ab_y;
    target_pose.pose.position.z = fly_ab_z;

    // 设置目标偏航角（四元数转换）
    target_pose.pose.orientation.x = 0.0;
    target_pose.pose.orientation.y = 0.0;
    target_pose.pose.orientation.z = sin(fly_ab_yaw / 2.0);
    target_pose.pose.orientation.w = cos(fly_ab_yaw / 2.0);

    // 飞行控制循环：持续发布目标位置直到到达或超时
    ros::Time start_time = ros::Time::now();
    const ros::Duration timeout(fly_ab_timeout_);  // 30秒超时时间

    while (ros::ok()) {

        // 更新消息时间戳
        target_pose.header.stamp = ros::Time::now();

        // 发布目标位置
        local_pos_pub.publish(target_pose);

        // 检查是否到达目标位置
        if (reachCheck(fly_ab_x, fly_ab_y, fly_ab_z, fly_ab_yaw)) {
            return true;
        }

        // 检查是否超时
        if ((ros::Time::now() - start_time) > timeout) {
            ROS_WARN_STREAM("Timeout while flying to target position. Time elapsed: " 
                          << (ros::Time::now() - start_time).toSec() << "s");
            return false;
        }

        ros::spinOnce();
        rate.sleep();
    }

    // 检查ROS节点是否正常运行
    if (!ros::ok()) {
        ROS_ERROR("ROS node is not running properly during flight");
        return false;
    }

    // ROS节点异常退出
    ROS_ERROR("ROS node shutdown during flight");
    return false;
}
