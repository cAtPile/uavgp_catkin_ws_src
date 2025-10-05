#include "avoid_planner_pkg/pointcloud_processor.h"

namespace avoid_planner {
    
/**
 * @brief 将角度值映射到直方图对应的网格索引
 * @details 根据输入角度的范围、直方图的角度区间和网格数量，计算该角度所属的直方图网格索引，
 *          同时处理角度越界和边界情况，确保索引的有效性
 * 
 * @param[in] angle 待映射的角度值（单位：弧度）
 * @param[in] min_angle 直方图角度范围的最小值（单位：弧度）
 * @param[in] max_angle 直方图角度范围的最大值（单位：弧度）
 * @param[in] num_bins 直方图在该角度维度上的网格数量（bins数）
 * 
 * @return 成功映射时返回非负的网格索引；若角度超出[min_angle, max_angle]范围，返回-1
 * 
 * @note 网格宽度（bin_width）通过角度总范围除以网格数量计算得出
 * @note 针对边界情况做了特殊处理：若计算出的索引小于0，返回0；若大于等于网格数量，返回最后一个网格索引（num_bins-1）
 * @note 该函数为通用工具函数，可同时用于方位角（azimuth）和仰角（elevation）的索引映射
 * 
 * @see generatePolarHistogram() 生成极坐标直方图时调用此函数映射方位角和仰角
 * @see updateHistogramFromPoint() 更新单个点到直方图时调用此函数获取网格索引
 * 
 * @warning 若输入的num_bins为0，会导致除法运算错误，需确保调用前num_bins已被正确初始化（至少为1）
 * @warning 若min_angle >= max_angle，角度范围无效，所有输入角度都会返回-1
 */
int PointcloudProcessor::angleToBinIndex(double angle, double min_angle, 
                                        double max_angle, size_t num_bins) {
    // 确保角度在范围内
    if (angle < min_angle || angle > max_angle) {
        return -1;
    }
    
    // 计算索引
    double range = max_angle - min_angle;
    double bin_width = range / num_bins;
    int index = static_cast<int>((angle - min_angle) / bin_width);
    
    // 处理边界情况
    if (index < 0) {
        return 0;
    }
    if (index >= static_cast<int>(num_bins)) {
        return num_bins - 1;
    }
    
    return index;
}
}