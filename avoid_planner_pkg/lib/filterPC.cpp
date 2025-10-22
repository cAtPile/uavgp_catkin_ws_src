/**
 * @file filterPC.cpp
 * @brief 过滤点云
 * @details pcl库
 * @author apoc
 * @date 2025/10/16
 */
#include"avoid_planner_pkg/avoid_planner.h"

 /**
 * @brief 加载并初始化传感器参数和直方图参数
 * @details 从ROS参数服务器读取配置参数，包括传感器范围、滤波参数、坐标系名称
 *          和直方图的角度分辨率等关键参数，并完成单位转换和直方图初始化
 * @param input
 * @param output
 * @see max_sensor_range_ 传感器最大探测范围(米)
 * @see min_sensor_range_ 传感器最小探测范围(米)  ？？？
 * @see voxel_grid_size_
 */
void AvoidPlanner::filterPC(const pcl::PointCloud<pcl::PointXYZ>& input, 
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