#include "avoid_planner_pkg/pointcloud_processor.h"
#include "avoid_planner_pkg/utils.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <limits>

namespace avoid_planner {
    
/**
 * @brief PointcloudProcessor类的构造函数
 * @details 初始化点云处理器的核心组件，包括ROS句柄、TF监听器、参数加载和点云订阅
 *          完成参数配置后会打印所有初始化参数，便于调试和确认配置正确性
 * 
 * @param[in] nh ROS节点句柄，用于创建订阅器和访问参数服务器
 * 
 * @note 构造函数初始化列表中完成了成员变量的初始化：
 *       - 复制ROS句柄(nh_)
 *       - 初始化TF缓冲区和监听器(tf_buffer_, tf_listener_)
 *       - 初始化更新标志(is_updated_)为false
 * 
 * @see loadParams() 用于从参数服务器加载配置参数
 * @see pointcloudCallback 点云消息的回调函数，由订阅器触发
 * @see pointcloud_sub_ 点云话题的订阅器成员变量
 * 
 * @warning 若参数加载失败，可能导致处理器工作在非预期状态
 * @warning 需确保TF变换在节点启动时已可用，否则可能影响后续坐标转换
 */
PointcloudProcessor::PointcloudProcessor(ros::NodeHandle& nh) : 
    nh_(nh), tf_listener_(tf_buffer_), is_updated_(false) {
    
    //导入参数
    loadParams();

    // 订阅点云话题
    pointcloud_sub_ = nh_.subscribe(lidar_topic_, 10,
                                   &PointcloudProcessor::pointcloudCallback, this);

    //参数展示
    ROS_INFO("PointcloudProcessor initialized with parameters:");
    ROS_INFO("  max_sensor_range: %.2f m", max_sensor_range_);
    ROS_INFO("  min_sensor_range: %.2f m", min_sensor_range_);
    ROS_INFO("  voxel_grid_size: %.2f m", voxel_grid_size_);
    ROS_INFO("  statistical_filter_mean_k: %d", statistical_filter_mean_k_);
    ROS_INFO("  statistical_filter_std_dev: %.2f", statistical_filter_std_dev_);
    ROS_INFO("  lidar_frame_id: %s", lidar_frame_id_.c_str());
    ROS_INFO("  body_frame_id: %s", body_frame_id_.c_str());
    ROS_INFO("  Histogram parameters:");
    ROS_INFO("    azimuth_resolution: %.4f rad (%.1f°)", histogram_.azimuth_resolution, 
             histogram_.azimuth_resolution * 180 / M_PI);
    ROS_INFO("    elevation_resolution: %.4f rad (%.1f°)", histogram_.elevation_resolution,
             histogram_.elevation_resolution * 180 / M_PI);
    ROS_INFO("    min_azimuth: %.2f rad (%.1f°)", histogram_.min_azimuth,
             histogram_.min_azimuth * 180 / M_PI);
    ROS_INFO("    max_azimuth: %.2f rad (%.1f°)", histogram_.max_azimuth,
             histogram_.max_azimuth * 180 / M_PI);
    ROS_INFO("    min_elevation: %.2f rad (%.1f°)", histogram_.min_elevation,
             histogram_.min_elevation * 180 / M_PI);
    ROS_INFO("    max_elevation: %.2f rad (%.1f°)", histogram_.max_elevation,
             histogram_.max_elevation * 180 / M_PI);
    ROS_INFO("    num_azimuth_bins: %zu", histogram_.num_azimuth_bins);
    ROS_INFO("    num_elevation_bins: %zu", histogram_.num_elevation_bins);

}

} // namespace avoid_planner