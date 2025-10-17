/**
 * @file generatePFdismap.cpp
 * @brief 生成dismap
 * @details 
 * @author apoc
 * @date 2025/10/17
 */
#include"avoid_planner_pkg/avoid_planner.h"

/**
 * @brief 根据机体坐标系下的点云生成极坐标直方图
 * @details 将点云中的有效点转换为极坐标（方位角、俯仰角和距离），
 *          并按角度分辨率划分到对应的直方图 bins 中，每个 bin 存储最近障碍物的距离
 * @param[in] cloud 机体坐标系下的点云数据（pcl::PointXYZ类型）
 * @see current_polar_field_ 存储极坐标直方图数据和参数的成员变量
 * @see updatePFpoint() 用于将单个点更新到直方图的内部函数
 * @see min_sensor_range_ 传感器最小有效距离阈值
 * @see max_sensor_range_ 传感器最大有效距离阈值
 */
void AvoidPlanner::generatePFdismap(const pcl::PointCloud<pcl::PointXYZ>& cloud) {

    // 重置直方图数据为无穷大(表示无障碍物)
    for (auto& az_bin : current_polar_field_.dis_map) {
        std::fill(az_bin.begin(), az_bin.end(), INFINITY);
    }
    
    // 处理每个点
    for (const auto& point : cloud) {
        // 忽略无效点
        if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
            continue;
        }
        
        // 检查距离是否在有效范围内
        double distance = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
        if (distance < min_sensor_range_ || distance > max_sensor_range_) {
            continue;
        }
        
        // 更新直方图
        updatePFpoint(point.x, point.y, point.z);
    }
}