#include "avoid_planner_pkg/pointcloud_processor.h"

namespace avoid_planner {
    
/**
 * @brief 将单个点的坐标转换为极坐标并更新直方图对应网格
 * @details 计算点在机体坐标系下的距离、方位角和仰角，
 *          映射到直方图的对应网格后，仅保留该网格中最近障碍物的距离
 * 
 * @param[in] x 点在机体坐标系下的x坐标(米)
 * @param[in] y 点在机体坐标系下的y坐标(米)
 * @param[in] z 点在机体坐标系下的z坐标(米)
 * 
 * @note 方位角(azimuth)定义：绕z轴旋转，x轴正方向为0，逆时针方向为正，范围[-π, π]
 * @note 仰角(elevation)定义：与xy平面的夹角，向上为正，范围[-π/2, π/2]
 * @note 每个直方图网格只存储最近障碍物的距离，确保避障决策基于最危险的障碍
 * 
 * @see angleToBinIndex() 将角度值映射到直方图网格索引的工具函数
 * @see histogram_ 存储极坐标直方图数据和参数的成员变量
 * 
 * @warning 若角度映射后的网格索引超出有效范围，该点将不参与直方图更新
 * @warning 距离计算使用欧氏距离，适用于大多数场景，但在特定地形下可能需要调整
 */
void PointcloudProcessor::updateHistogramFromPoint(double x, double y, double z) {
    // 计算距离
    double distance = std::sqrt(x*x + y*y + z*z);
    
    // 计算方位角 (绕z轴，x正方向为0，逆时针为正)
    double azimuth = std::atan2(y, x);  // 范围[-π, π]
    
    // 计算仰角 (与xy平面的夹角，向上为正)
    double elevation = std::atan2(z, std::sqrt(x*x + y*y));  // 范围[-π/2, π/2]
    
    // 映射到直方图网格
    int az_index = angleToBinIndex(azimuth, histogram_.min_azimuth, 
                                  histogram_.max_azimuth, histogram_.num_azimuth_bins);
    int el_index = angleToBinIndex(elevation, histogram_.min_elevation, 
                                  histogram_.max_elevation, histogram_.num_elevation_bins);
    
    // 检查索引有效性
    if (az_index < 0 || az_index >= static_cast<int>(histogram_.num_azimuth_bins) ||
        el_index < 0 || el_index >= static_cast<int>(histogram_.num_elevation_bins)) {
        return;
    }
    
    // 只保留每个网格中最近的障碍物
    if (distance < histogram_.data[az_index][el_index]) {
        histogram_.data[az_index][el_index] = distance;
    }
}


}