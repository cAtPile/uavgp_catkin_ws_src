#include "avoid_planner_pkg/pointcloud_processor.h"

namespace avoid_planner {
//创建直方图
/**
 * @brief 根据机体坐标系下的点云生成极坐标直方图
 * @details 将点云中的有效点转换为极坐标（方位角、俯仰角和距离），
 *          并按角度分辨率划分到对应的直方图 bins 中，每个 bin 存储最近障碍物的距离
 * 
 * @param[in] cloud 机体坐标系下的点云数据（pcl::PointXYZ类型）
 * 
 * @note 函数执行时会先锁定直方图互斥锁，确保多线程环境下的数据安全性
 * @note 直方图数据会被重置为无穷大(INFINITY)，表示初始状态下无障碍物
 * @note 仅处理有效点（非NaN值）和距离在传感器有效范围内的点
 * 
 * @see histogram_ 存储极坐标直方图数据和参数的成员变量
 * @see histogram_mutex_ 保护直方图数据的互斥锁
 * @see updateHistogramFromPoint() 用于将单个点更新到直方图的内部函数
 * @see min_sensor_range_ 传感器最小有效距离阈值
 * @see max_sensor_range_ 传感器最大有效距离阈值
 * 
 * @warning 若输入点云中包含大量无效点（NaN），可能影响直方图生成效率
 * @warning 极坐标转换可能存在数值计算误差，但在常规角度分辨率下可忽略
 */
void PointcloudProcessor::generatePolarHistogram(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    
    std::lock_guard<std::mutex> lock(histogram_mutex_);
    
    // 重置直方图数据为无穷大(表示无障碍物)
    for (auto& az_bin : histogram_.data) {
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
        updateHistogramFromPoint(point.x, point.y, point.z);
    }
}

}