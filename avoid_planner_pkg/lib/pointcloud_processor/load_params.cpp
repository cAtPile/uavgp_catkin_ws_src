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
void PointcloudProcessor::loadParams() {

    // 加载传感器参数
    nh_.param<double>("avoid_planner/max_sensor_range", max_sensor_range_, 50.0);
    nh_.param<double>("avoid_planner/min_sensor_range", min_sensor_range_, 0.5);
    nh_.param<double>("avoid_planner/voxel_grid_size", voxel_grid_size_, 0.1);
    nh_.param<int>("avoid_planner/statistical_filter_mean_k", statistical_filter_mean_k_, 10);
    nh_.param<double>("avoid_planner/statistical_filter_std_dev", statistical_filter_std_dev_, 0.1);
    nh_.param<std::string>("avoid_planner/lidar_frame_id", lidar_frame_id_, "livox_frame");
    nh_.param<std::string>("avoid_planner/body_frame_id", body_frame_id_, "base_link");
    nh_.param<std::string>("lidar_topic", lidar_topic_, "/livox/lidar");
    
    // 直方图参数 单位：度
    double az_res_deg;  //方位角步长（度）
    double el_res_deg;  //俯仰角步长
    double min_az_deg;  //最小方位角
    double max_az_deg;  //最大方位角
    double min_el_deg;  //最小俯仰角
    double max_el_deg;  //最大俯仰角
    
    // 加载角度参数
    nh_.param<double>("avoid_planner/azimuth_resolution_deg", az_res_deg, 1.0); // 默认1°
    nh_.param<double>("avoid_planner/elevation_resolution_deg", el_res_deg, 5.0); // 默认5°
    nh_.param<double>("avoid_planner/min_azimuth_deg", min_az_deg, -180.0); // 默认-180°
    nh_.param<double>("avoid_planner/max_azimuth_deg", max_az_deg, 180.0); // 默认180°
    nh_.param<double>("avoid_planner/min_elevation_deg", min_el_deg, -7.0); // 默认-7°
    nh_.param<double>("avoid_planner/max_elevation_deg", max_el_deg, 52.0); // 默认52°
    
    // 转换为弧度并更新直方图
    histogram_.azimuth_resolution = az_res_deg * M_PI / 180.0;
    histogram_.elevation_resolution = el_res_deg * M_PI / 180.0;
    histogram_.min_azimuth = min_az_deg * M_PI / 180.0;
    histogram_.max_azimuth = max_az_deg * M_PI / 180.0;
    histogram_.min_elevation = min_el_deg * M_PI / 180.0;
    histogram_.max_elevation = max_el_deg * M_PI / 180.0;
    
    // 计算bins数量
    histogram_.num_azimuth_bins = std::max(1UL, static_cast<size_t>(
        std::round((histogram_.max_azimuth - histogram_.min_azimuth) / histogram_.azimuth_resolution)
    ));
    histogram_.num_elevation_bins = std::max(1UL, static_cast<size_t>(
        std::round((histogram_.max_elevation - histogram_.min_elevation) / histogram_.elevation_resolution)
    ));
    
    // 重新初始化直方图数据
    histogram_.data.assign(
        histogram_.num_azimuth_bins,
        std::vector<double>(histogram_.num_elevation_bins, INFINITY)
    );
    
    // 最大范围同步
    histogram_.max_range = max_sensor_range_;
}

}