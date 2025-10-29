/**
 * @file generatePFpotmap.cpp
 * @brief
 * @details
 * @author
 * @date 2025/10/19
 */
#include"avoid_planner_pkg/avoid_planner.h"

/**
 * @brief
 * @details 
 * @param goal_az 目标方位角
 * @param goal_el 目标俯仰角
 * @param goal_dis 目标距离
 * @see calculateAtt根据目标计算引力位置和大小
 * @see calculateRep逐bin计算斥力
 * @see calculateTotal 计算引力斥力合力并存到pot_map
 * @see generateForceDir 生成合力方向
 */
void AvoidPlanner::generatePFpotmap(double goal_az, double goal_el, double goal_dis){
    
    // 获取势场网格尺寸
    size_t el_num = current_polar_field_.num_elevation_bins;
    size_t az_num = current_polar_field_.num_azimuth_bins;

    //将goal_az/el转换成bin
    int goal_az_idx = anglebinIndex(goal_az, 
                                  current_polar_field_.min_azimuth, 
                                  current_polar_field_.max_azimuth, 
                                  current_polar_field_.num_azimuth_bins);
    int goal_el_idx = anglebinIndex(goal_el, 
                                  current_polar_field_.min_elevation, 
                                  current_polar_field_.max_elevation, 
                                  current_polar_field_.num_elevation_bins);
    
    // 初始化势图存储容器
    pot_map_.resize(az_num, std::vector<double>(el_num, 0.0));
    
    // 遍历所有角度网格计算势场
    for (size_t el_idx = 0; el_idx < el_num; ++el_idx) {
        for (size_t az_idx = 0; az_idx < az_num; ++az_idx) {
            // 计算引力（仅目标方向有引力）
            double attractive_force = 0.0;
            if (az_idx == static_cast<size_t>(goal_az_idx) && el_idx == static_cast<size_t>(goal_el_idx) {
                attractive_force = calculateAtt(goal_dis);
            }
            
            // 计算斥力
            double repulsive_force = calculateRep(az_idx, el_idx);
            
            // 计算合力并存储到势图
            calculateTotal(az_idx, el_idx, attractive_force, repulsive_force);
        }
    }
    
    // 生成总体合力方向
    generateForceDir();
}