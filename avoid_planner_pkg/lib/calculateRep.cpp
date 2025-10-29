/**
 * @file calculateRep
 * @brief
 * @details
 * @author
 * @date 2025/10/19
 */
#include"avoid_planner_pkg/avoid_planner.h"

/**
 * @brief 计算指定角度网格的斥力大小
 * @details 基于障碍物距离计算斥力，距离越近斥力越大，避免碰撞
 * @param az_idx 方位角网格索引
 * @param el_idx 俯仰角网格索引
 * @return 斥力大小
 */
double AvoidPlanner::calculateRep(size_t az_idx, size_t el_idx) {
    // 斥力参数
    static double rep_k = 5.0;       // 斥力比例系数
    static double safe_dis = 5.0;    // 安全距离
    static double max_rep_force = 20.0;  // 最大斥力阈值

    // 从极坐标直方图获取当前网格的障碍物距离（假设存储在obstacle_distances中）
    // obstacle_distances[az_idx][el_idx]表示该角度方向上最近障碍物的距离
    double obs_dis = current_polar_field_.dis_map[az_idx][el_idx];

    // 若该方向无障碍物或大于安全距离，斥力为0
    if (obs_dis <= 0.0 || obs_dis >= safe_dis) {
        return 0.0;
    }

    // 斥力计算模型：基于距离倒数的非线性模型
    // 距离越近，斥力增长越快（F = k*(1/d - 1/safe) / d²，d为障碍物距离）
    double rep_force = rep_k * (1.0 / obs_dis - 1.0 / safe_dis) / (obs_dis * obs_dis);
    ROS_INFO("azBin: %d ,elBin: %d, obs_dis= %0.2f , rep_force=%0.2f",az_idx,el_idx,rep_force);

    // 确保斥力为正值
    if (rep_force < 0.0) {
        rep_force = 0.0;
    }

    // 限制最大斥力，避免因过近障碍物导致斥力过大
    if (rep_force > max_rep_force) {
        rep_force = max_rep_force;
    }

    return rep_force;
}