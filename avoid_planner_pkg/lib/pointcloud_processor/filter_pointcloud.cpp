#include "avoid_planner_pkg/pointcloud_processor.h"

namespace avoid_planner {
    
/**
 * @brief 加载并初始化传感器参数和直方图参数
 * @details 从ROS参数服务器读取配置参数，包括传感器范围、滤波参数、坐标系名称
 *          和直方图的角度分辨率等关键参数，并完成单位转换和直方图初始化
 * 
 * @note 参数加载使用ROS参数服务器的接口，若参数未配置则使用默认值
 * @note 角度参数在配置文件中以度为单位，加载后自动转换为弧度用于计算
 * @note 直方图的 bins 数量根据角度范围和分辨率自动计算，确保至少为1
 * @note 直方图数据被初始化为无穷大(INFINITY)，表示初始状态无障碍物
 * 
 * @see nh_ ROS节点句柄，用于访问参数服务器
 * @see histogram_ 存储直方图参数和数据的成员变量
 * @see max_sensor_range_ 传感器最大探测范围(米)
 * @see min_sensor_range_ 传感器最小探测范围(米)
 * 
 * @warning 若参数服务器中未配置相关参数，将使用默认值，可能与实际硬件不匹配
 * @warning 角度范围设置不合理可能导致直方图计算异常，建议保持默认范围或根据传感器特性调整
 */
void PointcloudProcessor::filterPointcloud(const pcl::PointCloud<pcl::PointXYZ>& input, 
                                          pcl::PointCloud<pcl::PointXYZ>& output) {
    
    pcl::PointCloud<pcl::PointXYZ> temp_cloud;
    
    // 1. 距离滤波 - 移除超出传感器范围的点
    pcl::PassThrough<pcl::PointXYZ> pass_filter;
    pass_filter.setInputCloud(input.makeShared());
    pass_filter.setFilterFieldName("x");
    pass_filter.setFilterLimits(-max_sensor_range_, max_sensor_range_);
    pass_filter.filter(temp_cloud);
    
    //是否空
    if (temp_cloud.empty()) {
        output = temp_cloud;
        return;
    }
    
    // 2. 体素网格滤波 - 下采样
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(temp_cloud.makeShared());
    voxel_filter.setLeafSize(voxel_grid_size_, voxel_grid_size_, voxel_grid_size_);
    voxel_filter.filter(temp_cloud);
    
    //
    if (temp_cloud.empty()) {
        output = temp_cloud;
        return;
    }
    
    // 3. 统计离群点去除
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> stat_filter;
    stat_filter.setInputCloud(temp_cloud.makeShared());
    stat_filter.setMeanK(statistical_filter_mean_k_);
    stat_filter.setStddevMulThresh(statistical_filter_std_dev_);
    stat_filter.filter(output);
}
}