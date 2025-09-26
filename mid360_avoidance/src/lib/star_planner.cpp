#include "mid360_avoidance/star_planner.h"
#include "mid360_avoidance/utils.h"
#include <cmath>
#include <queue>
#include <algorithm>
#include <ros/console.h>

namespace mid360_avoidance {

// 定义节点优先级比较器（用于优先队列）
struct NodePriorityCompare {
    bool operator()(const size_t& a_idx, const size_t& b_idx, 
                   const std::vector<StarNode>& search_tree) {
        // 总成本 = 累计成本 + 启发式成本 * 权重
        double cost_a = search_tree[a_idx].cost + 
                       search_tree[a_idx].heuristic * search_tree[a_idx].depth;
        double cost_b = search_tree[b_idx].cost + 
                       search_tree[b_idx].heuristic * search_tree[b_idx].depth;
        return cost_a > cost_b;  // 小成本优先
    }
};

StarPlanner::StarPlanner() : 
    has_valid_cost_matrix_(false),
    max_depth_(5), max_nodes_(50), step_size_(0.5),
    goal_tolerance_(0.3), heuristic_weight_(1.0),
    min_azimuth_(-M_PI), max_azimuth_(M_PI),
    min_elevation_(-0.2618), max_elevation_(0.2618) {}

void StarPlanner::setCostMatrix(const CostMatrix& cost_matrix) {
    std::lock_guard<std::mutex> lock(result_mutex_);
    cost_matrix_ = cost_matrix;
    has_valid_cost_matrix_ = true;
}

void StarPlanner::setStartAndGoal(const Eigen::Vector3d& start, const Eigen::Vector3d& goal) {
    std::lock_guard<std::mutex> lock(result_mutex_);
    start_position_ = start;
    goal_position_ = goal;
}

void StarPlanner::setCurrentVelocity(const Eigen::Vector3d& velocity) {
    std::lock_guard<std::mutex> lock(result_mutex_);
    current_velocity_ = velocity;
}

PathResult StarPlanner::searchPath() {
    std::lock_guard<std::mutex> lock(result_mutex_);
    PathResult result;
    result.timestamp = ros::Time::now();
    
    // 检查输入有效性
    if (!has_valid_cost_matrix_ || 
        start_position_.isZero() || 
        goal_position_.isZero() ||
        cost_matrix_.data.empty()) {
        ROS_WARN_THROTTLE(1.0, "Invalid input for star search");
        return result;
    }
    
    // 初始化搜索树
    search_tree_.clear();
    // 创建根节点（起点）
    Eigen::Vector3d initial_direction = current_velocity_.norm() > 1e-3 ? 
                                       current_velocity_.normalized() : 
                                       (goal_position_ - start_position_).normalized();
    StarNode root_node(
        start_position_,
        initial_direction,
        0.0,  // 初始成本
        calculateHeuristic(start_position_),  // 启发式成本
        -1,   // 无父节点
        0     // 深度为0
    );
    search_tree_.push_back(root_node);
    
    // 优先队列（按总成本排序的节点索引）
    std::priority_queue<size_t, std::vector<size_t>, 
                       std::function<bool(size_t, size_t)>> 
        priority_queue(
            [this](size_t a, size_t b) {
                double cost_a = search_tree_[a].cost + 
                               search_tree_[a].heuristic * heuristic_weight_;
                double cost_b = search_tree_[b].cost + 
                               search_tree_[b].heuristic * heuristic_weight_;
                return cost_a > cost_b;  // 小成本优先
            }
        );
    
    // 已访问节点集合
    std::set<size_t> visited_nodes;
    
    // 将根节点加入队列
    priority_queue.push(0);
    visited_nodes.insert(0);
    
    // 目标节点索引
    size_t goal_node_index = -1;
    bool goal_found = false;
    
    // 执行星形搜索
    while (!priority_queue.empty() && 
           search_tree_.size() < max_nodes_ && 
           !goal_found) {
        // 获取当前成本最小的节点
        size_t current_idx = priority_queue.top();
        priority_queue.pop();
        StarNode current_node = search_tree_[current_idx];
        
        // 检查是否到达目标
        if (isreachGoal(current_node.position)) {
            goal_node_index = current_idx;
            goal_found = true;
            break;
        }
        
        // 检查是否达到最大深度
        if (current_node.depth >= max_depth_) {
            continue;
        }
        
        // 生成子节点可能的方向
        std::vector<Eigen::Vector3d> child_directions = 
            generateChildDirections(current_node.direction);
        
        // 扩展子节点
        for (const auto& dir : child_directions) {
            // 计算子节点位置
            Eigen::Vector3d child_pos = current_node.position + dir * step_size_;
            
            // 计算子节点的成本
            size_t az_index, el_index;
            if (!directionToCostIndices(dir, az_index, el_index)) {
                continue;  // 方向超出成本矩阵范围，跳过
            }
            
            // 获取该方向的成本值
            double direction_cost = cost_matrix_.data[az_index][el_index];
            if (direction_cost >= 1.0) {
                continue;  // 成本过高（接近障碍物），跳过
            }
            
            // 计算累计成本（父节点成本 + 方向成本 * 步长）
            double child_cost = current_node.cost + direction_cost * step_size_;
            
            // 计算启发式成本
            double child_heuristic = calculateHeuristic(child_pos);
            
            // 创建子节点
            StarNode child_node(
                child_pos,
                dir,
                child_cost,
                child_heuristic,
                current_idx,
                current_node.depth + 1
            );
            
            // 添加到搜索树
            size_t child_idx = search_tree_.size();
            search_tree_.push_back(child_node);
            
            // 添加到优先队列
            if (visited_nodes.find(child_idx) == visited_nodes.end()) {
                priority_queue.push(child_idx);
                visited_nodes.insert(child_idx);
            }
            
            // 检查子节点是否到达目标
            if (isreachGoal(child_pos)) {
                goal_node_index = child_idx;
                goal_found = true;
                break;
            }
        }
    }
    
    // 处理搜索结果
    if (goal_found && goal_node_index != -1) {
        // 回溯路径
        std::vector<Eigen::Vector3d> raw_path = backtrackPath(goal_node_index);
        
        // 平滑路径
        std::vector<Eigen::Vector3d> smoothed_path = smoothPath(raw_path);
        
        // 填充结果
        result.waypoints = smoothed_path;
        result.success = true;
        result.total_cost = search_tree_[goal_node_index].cost;
        result.timestamp = ros::Time::now();
        
        ROS_DEBUG("Star search succeeded. Path length: %zu waypoints, Total cost: %.2f",
                 smoothed_path.size(), result.total_cost);
    } else {
        ROS_WARN_THROTTLE(1.0, "Star search failed. Nodes explored: %zu", search_tree_.size());
    }
    
    // 保存最后一次规划结果
    last_path_ = result;
    return result;
}

void StarPlanner::setParameters(size_t max_depth, size_t max_nodes, double step_size,
                               double goal_tolerance, double heuristic_weight) {
    std::lock_guard<std::mutex> lock(result_mutex_);
    max_depth_ = std::max(1ul, max_depth);
    max_nodes_ = std::max(10ul, max_nodes);
    step_size_ = std::max(0.1, step_size);
    goal_tolerance_ = std::max(0.1, goal_tolerance);
    heuristic_weight_ = std::max(0.1, heuristic_weight);
    
    ROS_INFO("StarPlanner parameters updated:");
    ROS_INFO("  max_depth: %zu, max_nodes: %zu", max_depth_, max_nodes_);
    ROS_INFO("  step_size: %.2f m, goal_tolerance: %.2f m", step_size_, goal_tolerance_);
    ROS_INFO("  heuristic_weight: %.2f", heuristic_weight_);
}

const PathResult& StarPlanner::getLastPath() const {
    std::lock_guard<std::mutex> lock(result_mutex_);
    return last_path_;
}

double StarPlanner::calculateHeuristic(const Eigen::Vector3d& node_position) {
    // 使用欧氏距离作为启发式成本（到目标点的直线距离）
    return calculateDistance(node_position, goal_position_);
}

bool StarPlanner::directionToCostIndices(const Eigen::Vector3d& direction,
                                        size_t& az_index, size_t& el_index) {
    // 计算方向向量的方位角和仰角
    double azimuth, elevation;
    directionToAngles(direction, azimuth, elevation);
    
    // 检查角度范围
    if (azimuth < min_azimuth_ || azimuth > max_azimuth_ ||
        elevation < min_elevation_ || elevation > max_elevation_) {
        return false;
    }
    
    // 计算角度分辨率
    double az_resolution = (max_azimuth_ - min_azimuth_) / cost_matrix_.num_azimuth_bins;
    double el_resolution = (max_elevation_ - min_elevation_) / cost_matrix_.num_elevation_bins;
    
    // 计算索引
    az_index = static_cast<size_t>((azimuth - min_azimuth_) / az_resolution);
    el_index = static_cast<size_t>((elevation - min_elevation_) / el_resolution);
    
    // 边界检查
    if (az_index >= cost_matrix_.num_azimuth_bins) {
        az_index = cost_matrix_.num_azimuth_bins - 1;
    }
    if (el_index >= cost_matrix_.num_elevation_bins) {
        el_index = cost_matrix_.num_elevation_bins - 1;
    }
    
    return true;
}

void StarPlanner::directionToAngles(const Eigen::Vector3d& direction,
                                   double& azimuth, double& elevation) {
    // 归一化方向向量
    Eigen::Vector3d dir = direction.normalized();
    
    // 计算方位角（绕z轴，x正方向为0，逆时针为正）
    azimuth = std::atan2(dir.y(), dir.x());
    azimuth = normalizeAngle(azimuth);  // 归一化到[-π, π]
    
    // 计算仰角（与xy平面的夹角，向上为正）
    double horizontal_dist = std::sqrt(dir.x() * dir.x() + dir.y() * dir.y());
    if (horizontal_dist < 1e-6) {
        elevation = dir.z() > 0 ? M_PI_2 : -M_PI_2;
    } else {
        elevation = std::atan2(dir.z(), horizontal_dist);
    }
}

std::vector<Eigen::Vector3d> StarPlanner::generateChildDirections(const Eigen::Vector3d& parent_direction) {
    std::vector<Eigen::Vector3d> directions;
    
    // 以父节点方向为中心，生成周围的可能方向（锥形采样）
    const int num_azimuth_steps = 8;   // 方位角采样数量
    const int num_elevation_steps = 3; // 仰角采样数量
    const double max_azimuth_delta = 0.5236;  // 最大方位角偏差（30度）
    const double max_elevation_delta = 0.2618; // 最大仰角偏差（15度）
    
    // 计算父方向的角度
    double parent_az, parent_el;
    directionToAngles(parent_direction, parent_az, parent_el);
    
    // 生成采样角度
    for (int i = 0; i < num_azimuth_steps; ++i) {
        for (int j = 0; j < num_elevation_steps; ++j) {
            // 计算相对偏差（中心方向偏差为0）
            double az_delta = max_azimuth_delta * (i - (num_azimuth_steps - 1)/2.0) / 
                            ((num_azimuth_steps - 1)/2.0);
            double el_delta = max_elevation_delta * (j - (num_elevation_steps - 1)/2.0) / 
                            ((num_elevation_steps - 1)/2.0);
            
            // 计算绝对角度
            double az = parent_az + az_delta;
            double el = parent_el + el_delta;
            
            // 归一化角度
            az = normalizeAngle(az);
            el = std::max(min_elevation_, std::min(max_elevation_, el));
            
            // 转换为方向向量
            double horizontal = std::cos(el);
            Eigen::Vector3d dir(
                horizontal * std::cos(az),
                horizontal * std::sin(az),
                std::sin(el)
            );
            
            directions.push_back(dir.normalized());
        }
    }
    
    // 去重（避免重复方向）
    std::sort(directions.begin(), directions.end(),
             [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                 return a.x() < b.x() || 
                        (a.x() == b.x() && a.y() < b.y()) ||
                        (a.x() == b.x() && a.y() == b.y() && a.z() < b.z());
             });
    auto last = std::unique(directions.begin(), directions.end(),
                           [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                               return (a - b).norm() < 1e-3;
                           });
    directions.erase(last, directions.end());
    
    return directions;
}

std::vector<Eigen::Vector3d> StarPlanner::backtrackPath(size_t goal_node_index) {
    std::vector<Eigen::Vector3d> path;
    
    // 从目标节点回溯到根节点
    size_t current_idx = goal_node_index;
    while (current_idx != -1 && current_idx < search_tree_.size()) {
        path.push_back(search_tree_[current_idx].position);
        current_idx = search_tree_[current_idx].parent_index;
    }
    
    // 反转路径（从起点到目标）
    std::reverse(path.begin(), path.end());
    
    return path;
}

bool StarPlanner::isreachGoal(const Eigen::Vector3d& node_position) {
    double distance = calculateDistance(node_position, goal_position_);
    return distance <= goal_tolerance_;
}

std::vector<Eigen::Vector3d> StarPlanner::smoothPath(const std::vector<Eigen::Vector3d>& path) {
    if (path.size() <= 2) {
        return path;  // 路径太短，无需平滑
    }
    
    // 使用简单滑动平均平滑路径
    std::vector<Eigen::Vector3d> smoothed = path;
    const size_t window_size = 3;
    
    // 跳过起点和终点，只平滑中间点
    for (size_t i = 1; i < path.size() - 1; ++i) {
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        size_t count = 0;
        
        // 计算窗口内的平均值
        for (int j = -static_cast<int>(window_size/2); j <= static_cast<int>(window_size/2); ++j) {
            size_t idx = i + j;
            if (idx >= 0 && idx < path.size()) {
                sum += path[idx];
                count++;
            }
        }
        
        if (count > 0) {
            smoothed[i] = sum / static_cast<double>(count);
        }
    }
    
    return smoothed;
}

} // namespace mid360_avoidance
