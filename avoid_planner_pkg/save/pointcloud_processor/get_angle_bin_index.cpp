#include "avoid_planner_pkg/pointcloud_processor.h"

/**
 * @brief 角度到直方图网格索引的转换接口函数
 * @details 作为公开接口，封装了内部的angleToBinIndex实现，
 *          提供与angleToBinIndex完全一致的角度到网格索引的映射功能
 * 
 * @param[in] angle 待转换的角度值（单位：弧度）
 * @param[in] min_angle 角度范围的最小值（单位：弧度）
 * @param[in] max_angle 角度范围的最大值（单位：弧度）
 * @param[in] num_bins 该角度维度上的直方图网格数量
 * 
 * @return 成功映射时返回非负的网格索引；若角度超出范围则返回-1
 * 
 * @note 该函数仅作为转发接口，实际转换逻辑由angleToBinIndex实现
 * @note 提供此接口便于外部模块复用角度到网格索引的转换逻辑，无需直接调用内部实现
 * 
 * @see angleToBinIndex() 实际执行角度到网格索引转换的内部函数
 * @see generatePolarHistogram() 内部生成直方图时使用的角度转换函数
 * 
 * @warning 与angleToBinIndex具有相同的约束：num_bins必须大于0，且min_angle < max_angle
 */
namespace avoid_planner {
    int PointcloudProcessor::getAngleBinIndex(double angle, double min_angle, double max_angle, size_t num_bins) {
    return angleToBinIndex(angle, min_angle, max_angle, num_bins);
}
}