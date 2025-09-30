#include "avoid_planner_pkg/local_planner.h"
#include "avoid_planner_pkg/utils.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/console.h>
#include <algorithm>

namespace mid360_avoidance {

/**
 * @brief 局部规划器构造函数（初始化核心模块和ROS接口）
 * @details 初始化局部规划器的所有子模块（点云处理、成本计算、路径搜索、MAVROS接口），
 * 订阅全局路径话题，创建局部路径发布者，并加载配置参数，是整个局部避障系统的入口
 * @param nh ROS节点句柄（用于创建订阅者、发布者和参数读取）
 */
LocalPlanner::LocalPlanner(ros::NodeHandle& nh) : 
    nh_(nh), is_running_(false), global_path_current_index_(0),
    planning_failure_count_(0), is_goal_reached_(false) {
    // 初始化核心子模块（使用智能指针管理生命周期）
    pointcloud_processor_ = std::make_unique<PointcloudProcessor>(nh);  // 点云处理器（障碍物检测）
    cost_calculator_ = std::make_unique<CostCalculator>();              // 成本计算器（方向成本评估）
    star_planner_ = std::make_unique<StarPlanner>();                    // 星形规划器（路径搜索）
    mavros_interface_ = std::make_unique<MavrosInterface>(nh);          // MAVROS接口（无人机控制）
    
    // 订阅全局路径话题（从全局规划器接收参考路径）
    global_path_sub_ = nh.subscribe("/global_path", 10, 
                                   &LocalPlanner::setGlobalPath, this);
    
    // 创建局部路径发布者（发布避障后的局部路径，用于可视化和调试）
    local_path_pub_ = nh.advertise<nav_msgs::Path>("/local_path", 10);
    
    // 从参数服务器加载配置参数（规划频率、安全距离等）
    loadParameters();
    
    // 输出初始化信息（包含规划更新频率）
    ROS_INFO("LocalPlanner initialized with update rate: %.1f Hz", config_.update_rate);
}

/**
 * @brief 启动局部规划器主循环
 * @details 启动规划器定时器，按配置的更新频率执行规划逻辑，仅在未运行状态下有效
 */
void LocalPlanner::start() {
    if (!is_running_) {  // 检查是否已运行（避免重复启动）
        is_running_ = true;  // 标记为运行状态
        // 创建定时器，按规划频率触发规划循环
        double period = 1.0 / config_.update_rate;  // 周期 = 1/频率（秒）
        planner_timer_ = nh_.createTimer(ros::Duration(period), 
                                        &LocalPlanner::plannerLoop, this);
        ROS_INFO("LocalPlanner started");
    }
}

/**
 * @brief 设置全局路径（从全局规划器接收新路径）
 * @details 接收并存储全局路径，重置路径索引、目标状态和失败计数器，是规划器的输入之一
 * @param global_path 指向全局路径消息的常量指针（nav_msgs::Path类型）
 */
void LocalPlanner::setGlobalPath(const nav_msgs::Path::ConstPtr& global_path) {
    std::lock_guard<std::mutex> lock(path_mutex_);  // 线程安全锁（保护全局路径读写）
    if (global_path->poses.empty()) {  // 检查路径是否为空
        ROS_WARN("Received empty global path");
        return;
    }
    
    global_path_ = *global_path;                  // 保存全局路径
    global_path_current_index_ = 0;               // 重置当前路径索引（从起点开始）
    is_goal_reached_ = false;                     // 重置目标到达标志
    planning_failure_count_ = 0;                  // 重置规划失败计数器
    
    // 输出接收全局路径的信息（包含路径点数量）
    ROS_INFO("Received new global path with %zu waypoints", global_path->poses.size());
}

/**
 * @brief 获取当前局部路径
 * @details 线程安全地返回最近一次规划的局部路径（用于外部模块查询）
 * @return nav_msgs::Path 局部路径消息（包含时间戳、坐标系和路径点）
 */
nav_msgs::Path LocalPlanner::getLocalPath() const {
    std::lock_guard<std::mutex> lock(path_mutex_);  // 线程安全锁
    return local_path_;
}

/**
 * @brief 检查局部规划器是否正在运行
 * @return bool 运行中返回true，已停止返回false
 */
bool LocalPlanner::isRunning() const {
    return is_running_;
}

/**
 * @brief 停止局部规划器主循环
 * @details 停止规划器定时器，标记为未运行状态，仅在运行中有效
 */
void LocalPlanner::stop() {
    if (is_running_) {  // 检查是否在运行（避免重复停止）
        is_running_ = false;  // 标记为停止状态
        planner_timer_.stop();  // 停止定时器（终止规划循环）
        ROS_INFO("LocalPlanner stopped");
    }
}

/**
 * @brief 规划器主循环（核心逻辑）
 * @details 按固定频率执行避障规划流程：检查状态→获取传感器数据→计算成本→搜索路径→发送控制指令，
 * 是局部规划器的核心执行函数
 * @param event 定时器事件（包含触发时间等信息）
 */
void LocalPlanner::plannerLoop(const ros::TimerEvent& event) {
    // 检查是否已到达全局目标（到达则停止规划）
    if (is_goal_reached_) {
        ROS_INFO_THROTTLE(5.0, "Global goal reached, stopping local planning");
        return;
    }
    
    // 检查是否有全局路径（无路径则跳过规划）
    {
        std::lock_guard<std::mutex> path_lock(path_mutex_);  // 局部作用域锁（自动释放）
        if (global_path_.poses.empty()) {
            ROS_WARN_THROTTLE(1.0, "No global path available, skipping planning");
            return;
        }
    }  // path_lock 在此自动解锁
    
    // 检查无人机连接状态（未连接则跳过规划）
    if (!mavros_interface_->isConnected()) {
        ROS_WARN_THROTTLE(1.0, "Not connected to vehicle, skipping planning");
        return;
    }
    
    // 获取无人机当前状态（位置、速度等）
    VehicleState vehicle_state = mavros_interface_->getVehicleState();
    Eigen::Vector3d current_pos = vehicle_state.position;  // 当前位置（ENU坐标系）
    Eigen::Vector3d current_vel = vehicle_state.velocity;  // 当前速度（ENU坐标系）
    
    // 1. 检查点云处理器是否有新数据（无新数据则跳过规划）
    if (!pointcloud_processor_->isUpdated()) {
        ROS_DEBUG_THROTTLE(1.0, "No new pointcloud data, skipping planning");
        return;
    }
    
    // 2. 获取处理后的障碍物直方图并传递给成本计算器
    const PolarHistogram& histogram = pointcloud_processor_->getHistogram();  // 障碍物分布直方图
    cost_calculator_->setHistogram(histogram);  // 设置成本计算器的障碍物数据
    pointcloud_processor_->resetUpdatedFlag();  // 重置点云更新标志（等待下一次数据）
    
    // 3. 计算局部目标点（从全局路径中截取的短期目标）
    Eigen::Vector3d local_goal = getLocalGoal();
    if (local_goal.isZero()) {  // 检查局部目标是否有效
        ROS_WARN_THROTTLE(1.0, "Failed to get local goal, skipping planning");
        handlePlanningFailure();  // 处理规划失败（计数+1，超过阈值则悬停）
        return;
    }
    
    // 4. 设置成本计算器参数并计算成本矩阵（核心步骤：评估各方向通行成本）
    cost_calculator_->setGoal(local_goal);                  // 设置目标点（用于目标方向成本）
    cost_calculator_->setRobotState(current_pos, current_vel);  // 设置无人机状态（位置和速度）
    if (!cost_calculator_->computeCostMatrix()) {  // 计算成本矩阵（各方向的综合成本）
        ROS_WARN_THROTTLE(1.0, "Failed to compute cost matrix, skipping planning");
        handlePlanningFailure();
        return;
    }
    
    // 5. 使用星形搜索算法生成局部避障路径
    const CostMatrix& cost_matrix = cost_calculator_->getCostMatrix();  // 获取成本矩阵
    star_planner_->setCostMatrix(cost_matrix);                          // 设置规划器的成本矩阵
    star_planner_->setStartAndGoal(current_pos, local_goal);            // 设置起点和局部目标
    star_planner_->setCurrentVelocity(current_vel);                      // 设置当前速度（影响初始方向）
    
    PathResult path_result = star_planner_->searchPath();  // 执行路径搜索
    if (!path_result.success || path_result.waypoints.empty()) {  // 检查路径是否有效
        ROS_WARN_THROTTLE(1.0, "Path planning failed, no valid path found");
        handlePlanningFailure();
        return;
    }
    
    // 6. 检查路径安全性（确保路径不靠近障碍物）
    if (!isPathSafe(path_result.waypoints)) {
        ROS_WARN_THROTTLE(1.0, "Generated path is unsafe, skipping");
        handlePlanningFailure();
        return;
    }
    
    // 7. 路径规划成功，重置失败计数器
    planning_failure_count_ = 0;
    
    // 8. 更新局部路径并发布（转换为ROS消息格式）
    {
        std::lock_guard<std::mutex> lock(path_mutex_);  // 线程安全锁
        local_path_.header.stamp = ros::Time::now();    // 设置时间戳
        local_path_.header.frame_id = global_path_.header.frame_id;  // 统一坐标系
        local_path_.poses.clear();  // 清空旧路径
        
        // 将路径点转换为ROS PoseStamped消息并添加到路径中
        for (const auto& waypoint : path_result.waypoints) {
            geometry_msgs::PoseStamped pose = vectorToPoseStamped(waypoint, local_path_.header.frame_id);
            local_path_.poses.push_back(pose);
        }
    }
    publishLocalPath();  // 发布局部路径（用于可视化）
    
    // 9. 将路径发送给无人机（仅在OFFBOARD模式且已武装时发送）
    if (mavros_interface_->isOffboardMode() && mavros_interface_->isArmed()) {
        mavros_interface_->sendPath(path_result.waypoints, local_path_.header.frame_id);
    }
    
    // 10. 检查是否到达局部目标（更新全局路径索引）
    double distance_to_goal = calculateDistance(current_pos, local_goal);  // 计算到局部目标的距离
    if (distance_to_goal < config_.path_safety_threshold) {  // 距离小于安全阈值视为到达
        std::lock_guard<std::mutex> lock(path_mutex_);  // 线程安全锁
        global_path_current_index_++;  // 全局路径索引+1（向目标前进）
        
        // 检查是否到达全局路径终点
        if (global_path_current_index_ >= global_path_.poses.size() - 1) {
            is_goal_reached_ = true;  // 标记全局目标已到达
            ROS_INFO("Global goal reached!");
        }
    }
}

/**
 * @brief 计算局部目标点（从全局路径中截取）
 * @details 从全局路径中找到距离当前位置一定距离的点作为局部目标，平衡路径跟踪和避障灵活性
 * @return Eigen::Vector3d 局部目标点坐标（ENU坐标系），无效时返回零向量
 */
Eigen::Vector3d LocalPlanner::getLocalGoal() {
    std::lock_guard<std::mutex> lock(path_mutex_);  // 线程安全锁（保护全局路径访问）
    
    // 获取无人机当前位置
    VehicleState vehicle_state = mavros_interface_->getVehicleState();
    Eigen::Vector3d current_pos = vehicle_state.position;
    
    // 找到距离当前位置最近的全局路径点（优化搜索范围，提高效率）
    double min_distance = INFINITY;
    size_t closest_index = global_path_current_index_;  // 从当前索引开始搜索
    
    // 限制搜索范围（当前索引附近5个点，避免全路径搜索）
    size_t search_range = std::min(5ul, global_path_.poses.size() - global_path_current_index_);
    for (size_t i = global_path_current_index_; 
         i < std::min(global_path_current_index_ + search_range, global_path_.poses.size()); ++i) {
        // 提取全局路径点坐标
        Eigen::Vector3d waypoint_pos(
            global_path_.poses[i].pose.position.x,
            global_path_.poses[i].pose.position.y,
            global_path_.poses[i].pose.position.z
        );
        // 计算与当前位置的距离
        double distance = calculateDistance(current_pos, waypoint_pos);
        if (distance < min_distance) {  // 更新最近点
            min_distance = distance;
            closest_index = i;
        }
    }
    
    global_path_current_index_ = closest_index;  // 更新当前索引为最近点
    
    // 从最近点向前搜索，找到距离满足局部目标距离的点
    Eigen::Vector3d local_goal = current_pos;  // 默认局部目标为当前位置（防止找不到时出错）
    for (size_t i = global_path_current_index_; i < global_path_.poses.size(); ++i) {
        // 提取全局路径点坐标
        Eigen::Vector3d waypoint_pos(
            global_path_.poses[i].pose.position.x,
            global_path_.poses[i].pose.position.y,
            global_path_.poses[i].pose.position.z
        );
        // 计算与当前位置的距离
        double distance = calculateDistance(current_pos, waypoint_pos);
        
        // 找到距离≥局部目标距离的点作为局部目标
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

/**
 * @brief 检查路径安全性（是否靠近障碍物）
 * @details 验证规划路径上的每个点是否与障碍物保持安全距离，避免碰撞风险
 * @param path 待检查的路径点列表（Eigen::Vector3d类型）
 * @return bool 路径安全返回true，存在碰撞风险返回false
 */
bool LocalPlanner::isPathSafe(const std::vector<Eigen::Vector3d>& path) {
    // 获取障碍物直方图（包含各方向障碍物距离）
    const PolarHistogram& histogram = pointcloud_processor_->getHistogram();
    
    // 检查路径上的每个点
    for (const auto& point : path) {
        // 获取无人机当前位置
        Eigen::Vector3d current_pos = mavros_interface_->getCurrentPosition();
        
        // 计算路径点相对于机体坐标系的位置
        Eigen::Vector3d point_rel = point - current_pos;
        double distance = point_rel.norm();  // 路径点与当前位置的距离
        
        if (distance < 1e-3) continue;  // 跳过当前位置（无意义）
        
        // 计算路径点相对当前位置的方位角（水平方向角度）
        double azimuth = std::atan2(point_rel.y(), point_rel.x());
        azimuth = normalizeAngle(azimuth);  // 归一化到[-π, π]
        
        // 计算路径点相对当前位置的仰角（垂直方向角度）
        double horizontal_dist = std::sqrt(point_rel.x()*point_rel.x() + point_rel.y()*point_rel.y());
        double elevation = std::atan2(point_rel.z(), horizontal_dist);
        
        // 将角度转换为直方图网格索引（找到对应方向的障碍物距离）
        int az_index = pointcloud_processor_->getAngleBinIndex(
            azimuth, histogram.min_azimuth, histogram.max_azimuth, histogram.num_azimuth_bins);
        int el_index = pointcloud_processor_->getAngleBinIndex(
            elevation, histogram.min_elevation, histogram.max_elevation, histogram.num_elevation_bins);
        
        // 检查索引有效性（超出范围则跳过）
        if (az_index < 0 || az_index >= static_cast<int>(histogram.num_azimuth_bins) ||
            el_index < 0 || el_index >= static_cast<int>(histogram.num_elevation_bins)) {
            continue;
        }
        
        // 检查该方向的障碍物距离是否小于安全阈值
        double obstacle_distance = histogram.data[az_index][el_index];
        if (obstacle_distance != INFINITY && obstacle_distance < config_.path_safety_threshold) {
            ROS_DEBUG("Path point is too close to obstacle (distance: %.2f m)", obstacle_distance);
            return false;  // 路径不安全
        }
    }
    
    return true;  // 路径安全
}

/**
 * @brief 发布局部路径（用于可视化）
 * @details 线程安全地发布最近规划的局部路径，供RViz等工具显示
 */
void LocalPlanner::publishLocalPath() {
    std::lock_guard<std::mutex> lock(path_mutex_);  // 线程安全锁
    if (!local_path_.poses.empty()) {  // 路径非空时发布
        local_path_pub_.publish(local_path_);
    }
}

/**
 * @brief 处理规划失败（计数和应急处理）
 * @details 累计规划失败次数，当超过阈值时发送悬停指令，确保无人机安全
 */
void LocalPlanner::handlePlanningFailure() {
    planning_failure_count_++;  // 失败计数+1
    ROS_WARN("Planning failure count: %d/%d", 
            planning_failure_count_, config_.max_failure_count);
    
    // 连续失败次数超过阈值，发送悬停指令（当前位置作为目标）
    if (planning_failure_count_ >= config_.max_failure_count) {
        ROS_ERROR("Max planning failures reached, sending hover command");
        Eigen::Vector3d current_pos = mavros_interface_->getCurrentPosition();
        mavros_interface_->sendTargetPosition(current_pos);  // 发送当前位置实现悬停
    }
}

/**
 * @brief 加载配置参数（从参数服务器）
 * @details 读取并验证规划器核心参数（频率、距离阈值等），并配置子模块参数
 */
void LocalPlanner::loadParameters() {
    // 从参数服务器加载局部规划器核心参数（带默认值）
    nh_.param<double>("update_rate", config_.update_rate, 10.0);  // 规划更新频率（Hz）
    nh_.param<double>("local_goal_distance", config_.local_goal_distance, 2.0);  // 局部目标距离（米）
    nh_.param<double>("path_safety_threshold", config_.path_safety_threshold, 0.5);  // 路径安全阈值（米）
    nh_.param<int>("max_failure_count", config_.max_failure_count, 3);  // 最大连续失败次数
    
    // 限制参数范围（确保合理性）
    config_.update_rate = std::max(1.0, std::min(50.0, config_.update_rate));  // 1-50Hz
    config_.local_goal_distance = std::max(0.5, config_.local_goal_distance);  // ≥0.5米
    config_.path_safety_threshold = std::max(0.2, config_.path_safety_threshold);  // ≥0.2米
    config_.max_failure_count = std::max(1, config_.max_failure_count);  // ≥1次
    
    // 加载子模块参数（成本计算器和星形规划器）
    double safety_distance, obstacle_weight, goal_weight, smoothness_weight;
    nh_.param<double>("safety_distance", safety_distance, 1.0);  // 安全距离（米）
    nh_.param<double>("obstacle_weight", obstacle_weight, 0.5);  // 障碍物成本权重
    nh_.param<double>("goal_weight", goal_weight, 0.3);          // 目标方向成本权重
    nh_.param<double>("smoothness_weight", smoothness_weight, 0.2);  // 平滑性成本权重

    int star_max_depth, star_max_nodes;  // 星形规划器参数（整数类型）
    double star_step_size, star_goal_tolerance, star_heuristic_weight;
    nh_.param<int>("star_max_depth", star_max_depth, 5);         // 最大搜索深度
    nh_.param<int>("star_max_nodes", star_max_nodes, 50);        // 最大搜索节点数
    nh_.param<double>("star_step_size", star_step_size, 0.5);    // 步长（米）
    nh_.param<double>("star_goal_tolerance", star_goal_tolerance, 0.3);  // 目标容差（米）
    nh_.param<double>("star_heuristic_weight", star_heuristic_weight, 1.0);  // 启发式权重


    // 配置成本计算器参数
    cost_calculator_->setParameters(
        safety_distance,
        obstacle_weight,
        goal_weight,
        smoothness_weight
    );

    // 配置星形规划器参数
    star_planner_->setParameters(
        star_max_depth,
        star_max_nodes,
        star_step_size,
        star_goal_tolerance,
        star_heuristic_weight
    );
} 

} // namespace mid360_avoidance