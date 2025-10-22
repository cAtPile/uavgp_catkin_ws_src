/**
 * @file
 * @brief
 * @details
 * @author
 * @date 2025/10/19
 */
#include"avoid_planner_pkg/avoid_planner.h"

/**
 * @brief 将单个点的坐标转换为极坐标并更新直方图对应网格
 * @details 计算点在机体坐标系下的距离、方位角和仰角，
 *          映射到直方图的对应网格后，仅保留该网格中最近障碍物的距离
 * @param[in] x 点在机体坐标系下的x坐标(米)
 * @param[in] y 点在机体坐标系下的y坐标(米)
 * @param[in] z 点在机体坐标系下的z坐标(米)
 * @see anglebinIndex() 将角度值映射到直方图网格索引的工具函数
 * @see current_polar_field_ 存储极坐标直方图数据和参数的成员变量
 */
void AvoidPlanner::updatePFpoint(double x, double y, double z) {

    // 计算距离
    double distance = std::sqrt(x*x + y*y + z*z);
    
    // 计算方位角 (绕z轴，x正方向为0，逆时针为正)
    double azimuth = std::atan2(y, x);  // 范围[-π, π]
    
    // 计算仰角 (与xy平面的夹角，向上为正)
    double elevation = std::atan2(z, std::sqrt(x*x + y*y));  // 范围[-π/2, π/2]
    
    // 映射到直方图网格
    int az_index = anglebinIndex(azimuth, current_polar_field_.min_azimuth, 
                                  current_polar_field_.max_azimuth, current_polar_field_.num_azimuth_bins);
    int el_index = anglebinIndex(elevation, current_polar_field_.min_elevation, 
                                  current_polar_field_.max_elevation, current_polar_field_.num_elevation_bins);
    
    // 检查索引有效性
    if (az_index < 0 || az_index >= static_cast<int>(current_polar_field_.num_azimuth_bins) ||
        el_index < 0 || el_index >= static_cast<int>(current_polar_field_.num_elevation_bins)) {
        return;
    }
    
    // 只保留每个网格中最近的障碍物
    if (distance < current_polar_field_.data[az_index][el_index]) {
        current_polar_field_.dis_map[az_index][el_index] = distance;
    }
}