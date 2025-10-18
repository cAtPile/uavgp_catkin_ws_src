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

//加载参数
bool loadParameters(ros::NodeHandle& nh, Params& params) {
    bool all_loaded = true;
    
    // 加载点云处理参数
    if (!nh.getParam("pointcloud_min_range", params.pointcloud_min_range)) {
        ROS_WARN("Using default value for pointcloud_min_range");
        all_loaded = false;
    }
    if (!nh.getParam("pointcloud_max_range", params.pointcloud_max_range)) {
        ROS_WARN("Using default value for pointcloud_max_range");
        all_loaded = false;
    }
    if (!nh.getParam("voxel_grid_size", params.voxel_grid_size)) {
        ROS_WARN("Using default value for voxel_grid_size");
        all_loaded = false;
    }
    if (!nh.getParam("statistical_filter_mean_k", params.statistical_filter_mean_k)) {
        ROS_WARN("Using default value for statistical_filter_mean_k");
        all_loaded = false;
    }
    if (!nh.getParam("statistical_filter_std_dev", params.statistical_filter_std_dev)) {
        ROS_WARN("Using default value for statistical_filter_std_dev");
        all_loaded = false;
    }
    if (!nh.getParam("lidar_frame_id", params.lidar_frame_id)) {
        ROS_WARN("Using default value for lidar_frame_id");
        all_loaded = false;
    }
    if (!nh.getParam("body_frame_id", params.body_frame_id)) {
        ROS_WARN("Using default value for body_frame_id");
        all_loaded = false;
    }
    
    // 加载成本计算参数
    if (!nh.getParam("safety_distance", params.safety_distance)) {
        ROS_WARN("Using default value for safety_distance");
        all_loaded = false;
    }
    if (!nh.getParam("obstacle_weight", params.obstacle_weight)) {
        ROS_WARN("Using default value for obstacle_weight");
        all_loaded = false;
    }
    if (!nh.getParam("goal_weight", params.goal_weight)) {
        ROS_WARN("Using default value for goal_weight");
        all_loaded = false;
    }
    if (!nh.getParam("smoothness_weight", params.smoothness_weight)) {
        ROS_WARN("Using default value for smoothness_weight");
        all_loaded = false;
    }
    
    // 加载星形搜索参数
    int max_depth, max_nodes;  // 使用int接收，再转换为size_t
    if (!nh.getParam("star_max_depth", max_depth)) {
        ROS_WARN("Using default value for star_max_depth");
        all_loaded = false;
    } else {
        params.star_max_depth = static_cast<size_t>(std::max(1, max_depth));
    }
    if (!nh.getParam("star_max_nodes", max_nodes)) {
        ROS_WARN("Using default value for star_max_nodes");
        all_loaded = false;
    } else {
        params.star_max_nodes = static_cast<size_t>(std::max(10, max_nodes));
    }
    if (!nh.getParam("star_step_size", params.star_step_size)) {
        ROS_WARN("Using default value for star_step_size");
        all_loaded = false;
    }
    if (!nh.getParam("star_goal_tolerance", params.star_goal_tolerance)) {
        ROS_WARN("Using default value for star_goal_tolerance");
        all_loaded = false;
    }
    if (!nh.getParam("star_heuristic_weight", params.star_heuristic_weight)) {
        ROS_WARN("Using default value for star_heuristic_weight");
        all_loaded = false;
    }
    
    // 加载局部规划器参数
    if (!nh.getParam("planner_update_rate", params.planner_update_rate)) {
        ROS_WARN("Using default value for planner_update_rate");
        all_loaded = false;
    }
    if (!nh.getParam("local_goal_distance", params.local_goal_distance)) {
        ROS_WARN("Using default value for local_goal_distance");
        all_loaded = false;
    }
    if (!nh.getParam("path_safety_threshold", params.path_safety_threshold)) {
        ROS_WARN("Using default value for path_safety_threshold");
        all_loaded = false;
    }
    if (!nh.getParam("max_failure_count", params.max_failure_count)) {
        ROS_WARN("Using default value for max_failure_count");
        all_loaded = false;
    }
    
    // 参数范围检查
    params.pointcloud_min_range = std::max(0.1, params.pointcloud_min_range);
    params.pointcloud_max_range = std::max(params.pointcloud_min_range + 0.5, params.pointcloud_max_range);
    params.voxel_grid_size = std::max(0.05, params.voxel_grid_size);
    params.statistical_filter_mean_k = std::max(1, params.statistical_filter_mean_k);
    params.statistical_filter_std_dev = std::max(0.01, params.statistical_filter_std_dev);
    params.safety_distance = std::max(0.3, params.safety_distance);
    params.obstacle_weight = std::max(0.0, std::min(1.0, params.obstacle_weight));
    params.goal_weight = std::max(0.0, std::min(1.0, params.goal_weight));
    params.smoothness_weight = std::max(0.0, std::min(1.0, params.smoothness_weight));
    params.star_step_size = std::max(0.2, params.star_step_size);
    params.star_goal_tolerance = std::max(0.1, params.star_goal_tolerance);
    params.star_heuristic_weight = std::max(0.1, params.star_heuristic_weight);
    params.planner_update_rate = std::max(1.0, std::min(50.0, params.planner_update_rate));
    params.local_goal_distance = std::max(0.5, params.local_goal_distance);
    params.path_safety_threshold = std::max(0.2, params.path_safety_threshold);
    params.max_failure_count = std::max(1, params.max_failure_count);
    
    return all_loaded;
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