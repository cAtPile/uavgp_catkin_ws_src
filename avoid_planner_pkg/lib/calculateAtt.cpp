/**
 * @file calculateAtt.cpp
 * @brief
 * @details
 * @author apoc
 * @date 2025/10/18
 */
#include"avoid_planner_pkg/avoid_planner.h"

/**
 * @brief 计算目标方向的引力大小
 * @details 基于目标距离计算引力，距离越远引力越大
 * @param goal_dis 目标与当前位置的距离
 * @return 引力大小
 */
double AvoidPlanner::calculateAtt(double goal_dis) {
    // 引力参数：比例系数
    static double att_k = 1.0;  // 默认比例系数，

    // 距离越远，引力越大，引导向目标运动
    double attractive_force = att_k * goal_dis;

    // 限制最大引力
    static double max_att_force = 10.0;  // 最大引力阈值，可配置
    if (attractive_force > max_att_force) {
        attractive_force = max_att_force;
    }

    return attractive_force;
}