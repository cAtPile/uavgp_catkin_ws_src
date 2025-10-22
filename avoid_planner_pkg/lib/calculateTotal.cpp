/**
 * @file calculateTotal
 * @brief
 * @details 
 * @author apoc
 * @date 2025/10/18
 */
#include "avoid_planner_pkg/avoid_planner.h"

/**
 * @brief 计算指定角度网格的引力与斥力合力
 * @details 合成引力和斥力，考虑方向特性（引力指向目标，斥力背离障碍物），得到总力并更新势场图
 * @param az_idx 方位角网格索引
 * @param el_idx 俯仰角网格索引
 * @param att_force 该网格的引力大小
 * @param rep_force 该网格的斥力大小
 */
void AvoidPlanner::calculateTotal(size_t az_idx, size_t el_idx, double att_force, double rep_force) {
    // 合力计算：引力（指向目标为正）与斥力（背离障碍物为正）的矢量叠加
    double total_force = att_force - rep_force;  // 斥力抵消引力，实现避障

    // 边界处理：确保合力不为负
    if (total_force < 0.0) {
        total_force = 0.0;
    }

    // 将合力存储到势场图对应网格
    current_polar_field_.pot_map[az_idx][el_idx] = total_force;

}