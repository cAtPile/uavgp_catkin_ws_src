#include "local_planner.h"
#include <ros/ros.h>
#include <cmath>
#include <algorithm>

namespace avoidance {

LocalPlanner::LocalPlanner() : has_goal_(false), has_cloud_(false) {
    // 初始化默认参数
    setParams();
    max_avoidance_distance_ = 1.0f; // 最大避障偏移距离
}

void LocalPlanner::setCurrentPosition(const Eigen::Vector3f& pos) {
    current_pos_ = pos;
}

void LocalPlanner::setGoal(const Eigen::Vector3f& goal) {
    goal_pos_ = goal;
    has_goal_ = true;
}

void LocalPlanner::setObstacleCloud(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
    obstacle_cloud_ = cloud;
    has_cloud_ = true;
}

void LocalPlanner::setParams(float safe_distance, float step_size, float goal_tolerance) {
    safe_distance_ = safe_distance;
    step_size_ = step_size;
    goal_tolerance_ = goal_tolerance;
}

PlannerOutput LocalPlanner::computeNextSetpoint() {
    PlannerOutput output;
    output.planning_time = ros::Time::now();

    // 检查是否已设置目标点和点云数据
    if (!has_goal_ || !has_cloud_) {
        ROS_WARN_THROTTLE(1.0, "Missing goal or obstacle cloud data");
        output.next_setpoint = current_pos_; // 保持当前位置
        output.is_goal_reached = false;
        output.distance_to_goal = distance(current_pos_, goal_pos_);
        return output;
    }

    // 计算当前位置到目标点的距离
    float dist_to_goal = distance(current_pos_, goal_pos_);
    output.distance_to_goal = dist_to_goal;

    // 检查是否已到达目标点
    if (dist_to_goal < goal_tolerance_) {
        output.next_setpoint = goal_pos_;
        output.is_goal_reached = true;
        return output;
    }

    // 计算朝向目标点的单位向量
    Eigen::Vector3f direction = (goal_pos_ - current_pos_).normalized();
    
    // 计算理想下一步位置（不考虑障碍物）
    Eigen::Vector3f ideal_next = current_pos_ + direction * std::min(step_size_, dist_to_goal);

    // 检查直线路径是否通畅
    if (!isPathBlocked(current_pos_, ideal_next)) {
        output.next_setpoint = ideal_next;
        output.is_goal_reached = false;
        return output;
    }

    // 路径被阻挡，生成并评估避障备选点
    std::vector<Eigen::Vector3f> candidates = generateAvoidanceCandidates();
    if (candidates.empty()) {
        ROS_WARN("No valid avoidance candidates found");
        output.next_setpoint = current_pos_; // 无法避障，停在原地
        output.is_goal_reached = false;
        return output;
    }

    // 选择最优备选点
    int best_idx = evaluateCandidates(candidates);
    output.next_setpoint = candidates[best_idx];
    output.is_goal_reached = false;

    return output;
}

bool LocalPlanner::isPathBlocked(const Eigen::Vector3f& start, const Eigen::Vector3f& end) {
    Eigen::Vector3f path_vec = end - start;
    float path_length = path_vec.norm();
    if (path_length < 0.001f) return false; // 路径过短，视为无障碍物

    Eigen::Vector3f path_dir = path_vec.normalized();

    // 检查每个障碍物点是否在路径附近
    for (const auto& point : obstacle_cloud_.points) {
        Eigen::Vector3f obs_pos(point.x, point.y, point.z);
        
        // 计算障碍物到路径起点的向量
        Eigen::Vector3f obs_vec = obs_pos - start;
        
        // 计算障碍物在路径上的投影比例
        float proj_ratio = obs_vec.dot(path_dir) / path_length;
        
        // 投影在路径段范围内（0到1之间）
        if (proj_ratio >= 0.0f && proj_ratio <= 1.0f) {
            // 计算障碍物到路径的垂直距离
            Eigen::Vector3f closest_point = start + path_dir * proj_ratio * path_length;
            float dist = distance(obs_pos, closest_point);
            
            // 如果距离小于安全距离，视为路径被阻挡
            if (dist < safe_distance_) {
                return true;
            }
        }
    }

    return false;
}

std::vector<Eigen::Vector3f> LocalPlanner::generateAvoidanceCandidates() {
    std::vector<Eigen::Vector3f> candidates;

    // 计算朝向目标点的方向
    Eigen::Vector3f goal_dir = (goal_pos_ - current_pos_).normalized();
    
    // 生成垂直于目标方向的两个正交向量（用于避障偏移）
    Eigen::Vector3f ortho1, ortho2;
    
    // 找到一个不与目标方向共线的向量来生成正交向量
    if (std::fabs(goal_dir.z()) < 0.9f) {
        ortho1 = goal_dir.cross(Eigen::Vector3f(0, 0, 1)).normalized();
    } else {
        ortho1 = goal_dir.cross(Eigen::Vector3f(1, 0, 0)).normalized();
    }
    ortho2 = goal_dir.cross(ortho1).normalized();

    // 生成多个备选点（不同方向和距离的偏移）
    const int num_angles = 8; // 角度数量
    const int num_distances = 2; // 距离数量
    
    for (int i = 0; i < num_angles; ++i) {
        float angle = 2.0f * M_PI * static_cast<float>(i) / num_angles;
        
        for (int j = 1; j <= num_distances; ++j) {
            float dist_ratio = static_cast<float>(j) / num_distances;
            float avoidance_dist = max_avoidance_distance_ * dist_ratio;
            
            // 计算偏移方向
            Eigen::Vector3f avoid_dir = ortho1 * std::cos(angle) + ortho2 * std::sin(angle);
            avoid_dir.normalize();
            
            // 计算备选点位置
            Eigen::Vector3f candidate = current_pos_ + 
                                      goal_dir * step_size_ * 0.5f + // 向前移动一半步长
                                      avoid_dir * avoidance_dist;     // 加上避障偏移
            
            // 检查备选点是否在障碍物中
            bool in_obstacle = false;
            for (const auto& point : obstacle_cloud_.points) {
                Eigen::Vector3f obs_pos(point.x, point.y, point.z);
                if (distance(candidate, obs_pos) < safe_distance_ * 0.5f) {
                    in_obstacle = true;
                    break;
                }
            }
            
            if (!in_obstacle) {
                candidates.push_back(candidate);
            }
        }
    }

    return candidates;
}

int LocalPlanner::evaluateCandidates(const std::vector<Eigen::Vector3f>& candidates) {
    if (candidates.empty()) return -1;

    int best_idx = 0;
    float best_score = -INFINITY;

    for (size_t i = 0; i < candidates.size(); ++i) {
        const auto& candidate = candidates[i];
        
        // 1. 距离目标点越近得分越高
        float dist_to_goal = distance(candidate, goal_pos_);
        float goal_score = 1.0f / (1.0f + dist_to_goal);
        
        // 2. 与当前位置的距离接近步长得分高
        float step_dist = distance(current_pos_, candidate);
        float step_score = 1.0f / (1.0f + std::fabs(step_dist - step_size_));
        
        // 3. 远离障碍物得分高
        float min_obs_dist = INFINITY;
        for (const auto& point : obstacle_cloud_.points) {
            Eigen::Vector3f obs_pos(point.x, point.y, point.z);
            float dist = distance(candidate, obs_pos);
            if (dist < min_obs_dist) {
                min_obs_dist = dist;
            }
        }
        float obstacle_score = min_obs_dist / (1.0f + min_obs_dist);
        
        // 综合得分（权重可根据实际情况调整）
        float total_score = 0.5f * goal_score + 0.2f * step_score + 0.3f * obstacle_score;
        
        // 更新最佳候选点
        if (total_score > best_score) {
            best_score = total_score;
            best_idx = i;
        }
    }

    return best_idx;
}

float LocalPlanner::distance(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
    return (a - b).norm();
}

}  // namespace avoidance
    