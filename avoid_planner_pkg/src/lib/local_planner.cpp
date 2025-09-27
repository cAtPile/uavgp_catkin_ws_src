#include "mid360_avoidance/local_planner.h"
#include "mid360_avoidance/utils.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/console.h>
#include <algorithm>

namespace mid360_avoidance {

LocalPlanner::LocalPlanner(ros::NodeHandle& nh) : 
    nh_(nh), is_running_(false), global_path_current_index_(0),
    planning_failure_count_(0), is_goal_reached_(false) {
    // 初始化子模块
    pointcloud_processor_ = std::make_unique<PointcloudProcessor>(nh);
    cost_calculator_ = std::make_unique<CostCalculator>();
    star_planner_ = std::make_unique<StarPlanner>();
    mavros_interface_ = std::make_unique<MavrosInterface>(nh);
    
    // 订阅全局路径
    global_path_sub_ = nh.subscribe("/global_path", 10, 
                                   &LocalPlanner::setGlobalPath, this);
    
    // 发布局部路径
    local_path_pub_ = nh.advertise<nav_msgs::Path>("/local_path", 10);
    
    // 加载配置参数
    loadParameters();
    
    ROS_INFO("LocalPlanner initialized with update rate: %.1f Hz", config_.update_rate);
}

void LocalPlanner::start() {
    if (!is_running_) {
        is_running_ = true;
        // 启动规划器主循环
        double period = 1.0 / config_.update_rate;
        planner_timer_ = nh_.createTimer(ros::Duration(period), 
                                        &LocalPlanner::plannerLoop, this);
        ROS_INFO("LocalPlanner started");
    }
}

void LocalPlanner::setGlobalPath(const nav_msgs::Path::ConstPtr& global_path) {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (global_path->poses.empty()) {
        ROS_WARN("Received empty global path");
        return;
    }
    
    global_path_ = *global_path;
    global_path_current_index_ = 0;
    is_goal_reached_ = false;
    planning_failure_count_ = 0;
    
    ROS_INFO("Received new global path with %zu waypoints", global_path->poses.size());
}

nav_msgs::Path LocalPlanner::getLocalPath() const {
    std::lock_guard<std::mutex> lock(path_mutex_);
    return local_path_;
}

bool LocalPlanner::isRunning() const {
    return is_running_;
}

void LocalPlanner::stop() {
    if (is_running_) {
        is_running_ = false;
        planner_timer_.stop();
        ROS_INFO("LocalPlanner stopped");
    }
}

void LocalPlanner::plannerLoop() {
    // 检查是否已到达全局目标
    if (is_goal_reached_) {
        ROS_INFO_THROTTLE(5.0, "Global goal reached, stopping local planning");
        return;
    }
    
    // 检查是否有全局路径
    std::lock_guard<std::mutex> path_lock(path_mutex_);
    if (global_path_.poses.empty()) {
        ROS_WARN_THROTTLE(1.0, "No global path available, skipping planning");
        return;
    }
    path_lock.unlock();
    
    // 检查无人机连接状态
    if (!mavros_interface_->isConnected()) {
        ROS_WARN_THROTTLE(1.0, "Not connected to vehicle, skipping planning");
        return;
    }
    
    // 获取无人机当前状态
    VehicleState vehicle_state = mavros_interface_->getVehicleState();
    Eigen::Vector3d current_pos = vehicle_state.position;
    Eigen::Vector3d current_vel = vehicle_state.velocity;
    
    // 1. 检查点云处理器是否有新数据
    if (!pointcloud_processor_->isUpdated()) {
        ROS_DEBUG_THROTTLE(1.0, "No new pointcloud data, skipping planning");
        return;
    }
    
    // 2. 获取处理后的障碍物直方图
    const PolarHistogram& histogram = pointcloud_processor_->getHistogram();
    cost_calculator_->setHistogram(histogram);
    pointcloud_processor_->resetUpdatedFlag();
    
    // 3. 获取局部目标点
    Eigen::Vector3d local_goal = getLocalGoal();
    if (local_goal.isZero()) {
        ROS_WARN_THROTTLE(1.0, "Failed to get local goal, skipping planning");
        handlePlanningFailure();
        return;
    }
    
    // 4. 设置成本计算器参数并计算成本矩阵
    cost_calculator_->setGoal(local_goal);
    cost_calculator_->setRobotState(current_pos, current_vel);
    if (!cost_calculator_->computeCostMatrix()) {
        ROS_WARN_THROTTLE(1.0, "Failed to compute cost matrix, skipping planning");
        handlePlanningFailure();
        return;
    }
    
    // 5. 使用星形搜索生成局部路径
    const CostMatrix& cost_matrix = cost_calculator_->getCostMatrix();
    star_planner_->setCostMatrix(cost_matrix);
    star_planner_->setStartAndGoal(current_pos, local_goal);
    star_planner_->setCurrentVelocity(current_vel);
    
    PathResult path_result = star_planner_->searchPath();
    if (!path_result.success || path_result.waypoints.empty()) {
        ROS_WARN_THROTTLE(1.0, "Path planning failed, no valid path found");
        handlePlanningFailure();
        return;
    }
    
    // 6. 检查路径安全性
    if (!isPathSafe(path_result.waypoints)) {
        ROS_WARN_THROTTLE(1.0, "Generated path is unsafe, skipping");
        handlePlanningFailure();
        return;
    }
    
    // 7. 路径规划成功，重置失败计数器
    planning_failure_count_ = 0;
    
    // 8. 更新局部路径并发布
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        local_path_.header.stamp = ros::Time::now();
        local_path_.header.frame_id = global_path_.header.frame_id;
        local_path_.poses.clear();
        
        // 转换路径点为ROS消息
        for (const auto& waypoint : path_result.waypoints) {
            geometry_msgs::PoseStamped pose = vectorToPoseStamped(waypoint, local_path_.header.frame_id);
            local_path_.poses.push_back(pose);
        }
    }
    publishLocalPath();
    
    // 9. 将路径发送给PX4
    if (mavros_interface_->isOffboardMode() && mavros_interface_->isArmed()) {
        mavros_interface_->sendPath(path_result.waypoints, local_path_.header.frame_id);
    }
    
    // 10. 检查是否到达局部目标
    double distance_to_goal = calculateDistance(current_pos, local_goal);
    if (distance_to_goal < config_.path_safety_threshold) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        global_path_current_index_++;
        
        // 检查是否到达全局目标
        if (global_path_current_index_ >= global_path_.poses.size() - 1) {
            is_goal_reached_ = true;
            ROS_INFO("Global goal reached!");
        }
    }
}

Eigen::Vector3d LocalPlanner::getLocalGoal() {
    std::lock_guard<std::mutex> lock(path_mutex_);
    
    // 找到距离当前位置最近的全局路径点
    VehicleState vehicle_state = mavros_interface_->getVehicleState();
    Eigen::Vector3d current_pos = vehicle_state.position;
    
    double min_distance = INFINITY;
    size_t closest_index = global_path_current_index_;
    
    // 在当前索引附近搜索最近点（提高效率）
    size_t search_range = std::min(5ul, global_path_.poses.size() - global_path_current_index_);
    for (size_t i = global_path_current_index_; 
         i < std::min(global_path_current_index_ + search_range, global_path_.poses.size()); ++i) {
        Eigen::Vector3d waypoint_pos(
            global_path_.poses[i].pose.position.x,
            global_path_.poses[i].pose.position.y,
            global_path_.poses[i].pose.position.z
        );
        double distance = calculateDistance(current_pos, waypoint_pos);
        if (distance < min_distance) {
            min_distance = distance;
            closest_index = i;
        }
    }
    
    global_path_current_index_ = closest_index;
    
    // 从当前点向前搜索，找到距离超过局部目标距离的点作为局部目标
    Eigen::Vector3d local_goal = current_pos;
    for (size_t i = global_path_current_index_; i < global_path_.poses.size(); ++i) {
        Eigen::Vector3d waypoint_pos(
            global_path_.poses[i].pose.position.x,
            global_path_.poses[i].pose.position.y,
            global_path_.poses[i].pose.position.z
        );
        double distance = calculateDistance(current_pos, waypoint_pos);
        
        if (distance >= config_.local_goal_distance) {
            local_goal = waypoint_pos;
            break;
        }
        
        // 如果到达全局路径终点，将终点设为局部目标
        if (i == global_path_.poses.size() - 1) {
            local_goal = waypoint_pos;
            break;
        }
    }
    
    return local_goal;
}

bool LocalPlanner::isPathSafe(const std::vector<Eigen::Vector3d>& path) {
    // 获取障碍物直方图
    const PolarHistogram& histogram = pointcloud_processor_->getHistogram();
    
    // 检查路径上的每个点是否与障碍物距离过近
    for (const auto& point : path) {
        // 获取无人机当前位置
        Eigen::Vector3d current_pos = mavros_interface_->getCurrentPosition();
        
        // 计算路径点相对于机体坐标系的位置
        Eigen::Vector3d point_rel = point - current_pos;
        double distance = point_rel.norm();
        
        if (distance < 1e-3) continue;  // 跳过当前位置
        
        // 计算相对方向角
        double azimuth = std::atan2(point_rel.y(), point_rel.x());
        azimuth = normalizeAngle(azimuth);
        
        double horizontal_dist = std::sqrt(point_rel.x()*point_rel.x() + point_rel.y()*point_rel.y());
        double elevation = std::atan2(point_rel.z(), horizontal_dist);
        
        // 找到对应的直方图网格
        int az_index = pointcloud_processor_->angleToBinIndex(
            azimuth, histogram.min_azimuth, histogram.max_azimuth, histogram.num_azimuth_bins);
        int el_index = pointcloud_processor_->angleToBinIndex(
            elevation, histogram.min_elevation, histogram.max_elevation, histogram.num_elevation_bins);
        
        // 检查索引有效性
        if (az_index < 0 || az_index >= static_cast<int>(histogram.num_azimuth_bins) ||
            el_index < 0 || el_index >= static_cast<int>(histogram.num_elevation_bins)) {
            continue;
        }
        
        // 检查与障碍物的距离是否小于安全阈值
        double obstacle_distance = histogram.data[az_index][el_index];
        if (obstacle_distance != INFINITY && obstacle_distance < config_.path_safety_threshold) {
            ROS_DEBUG("Path point is too close to obstacle (distance: %.2f m)", obstacle_distance);
            return false;
        }
    }
    
    return true;
}

void LocalPlanner::publishLocalPath() {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (!local_path_.poses.empty()) {
        local_path_pub_.publish(local_path_);
    }
}

void LocalPlanner::handlePlanningFailure() {
    planning_failure_count_++;
    ROS_WARN("Planning failure count: %d/%d", 
            planning_failure_count_, config_.max_failure_count);
    
    // 如果连续失败次数超过阈值，发送悬停指令
    if (planning_failure_count_ >= config_.max_failure_count) {
        ROS_ERROR("Max planning failures reached, sending hover command");
        Eigen::Vector3d current_pos = mavros_interface_->getCurrentPosition();
        mavros_interface_->sendTargetPosition(current_pos);  // 发送当前位置作为目标，实现悬停
    }
}

void LocalPlanner::loadParameters() {
    // 从参数服务器加载配置
    nh_.param<double>("update_rate", config_.update_rate, 10.0);
    nh_.param<double>("local_goal_distance", config_.local_goal_distance, 2.0);
    nh_.param<double>("path_safety_threshold", config_.path_safety_threshold, 0.5);
    nh_.param<int>("max_failure_count", config_.max_failure_count, 3);
    
    // 限制参数范围
    config_.update_rate = std::max(1.0, std::min(50.0, config_.update_rate));
    config_.local_goal_distance = std::max(0.5, config_.local_goal_distance);
    config_.path_safety_threshold = std::max(0.2, config_.path_safety_threshold);
    config_.max_failure_count = std::max(1, config_.max_failure_count);
    
    // 配置子模块参数
    Params params;
    if (loadParameters(nh_, params)) {
        // 配置成本计算器
        cost_calculator_->setParameters(
            params.safety_distance,
            params.obstacle_weight,
            params.goal_weight,
            params.smoothness_weight
        );
        
        // 配置星形规划器
        star_planner_->setParameters(
            params.star_max_depth,
            params.star_max_nodes,
            params.star_step_size,
            params.star_goal_tolerance,
            params.star_heuristic_weight
        );
    }
}

} // namespace mid360_avoidance
