/**
 * @file utils.cpp
 * @brief 工具函数
 * @details
 * @author apoc
 * @date 2025/10/18
 */
#include "avoid_planner_pkg/utils.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <ros/console.h>

//rpy
Eigen::Vector3d quaternionToYaw(const geometry_msgs::Quaternion& quat) {
    // 从四元数提取RPY角
    tf2::Quaternion q(
        quat.x,
        quat.y,
        quat.z,
        quat.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    // 归一化偏航角到[-π, π]
    yaw = normalizeAngle(yaw);
    
    // 返回只包含偏航角的向量（x,y为0）
    return Eigen::Vector3d(0.0, 0.0, yaw);
}

//欧式距离
double calculateDistance(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    // 计算欧氏距离
    return (a - b).norm();
}

//平滑路径
void smoothPath(std::vector<Eigen::Vector3d>& path, size_t window_size) {
    if (path.size() <= 2 || window_size < 3) {
        return;  // 路径太短或窗口太小，无需平滑
    }
    
    // 确保窗口大小为奇数
    window_size = window_size % 2 == 0 ? window_size + 1 : window_size;
    const size_t half_window = window_size / 2;
    
    // 保存原始路径用于计算
    std::vector<Eigen::Vector3d> original = path;
    
    // 平滑路径（保留起点和终点）
    for (size_t i = 1; i < path.size() - 1; ++i) {
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        size_t count = 0;
        
        // 计算窗口内的平均值
        for (int j = -static_cast<int>(half_window); j <= static_cast<int>(half_window); ++j) {
            size_t idx = i + j;
            if (idx < path.size()) {  // 允许左侧超出边界时从起点开始
                sum += original[idx];
                count++;
            }
        }
        
        if (count > 0) {
            path[i] = sum / static_cast<double>(count);
        }
    }
}

//归一化角度
double normalizeAngle(double angle) {
    // 将角度归一化到[-π, π]范围
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

//设置位姿
geometry_msgs::PoseStamped vectorToPoseStamped(const Eigen::Vector3d& position,
                                              const std::string& frame_id) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id;
    
    // 设置位置
    pose.pose.position.x = position.x();
    pose.pose.position.y = position.y();
    pose.pose.position.z = position.z();
    
    // 设置姿态（默认无旋转，可根据需要修改）
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    
    return pose;
}