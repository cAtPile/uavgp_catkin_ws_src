#include "avoid_planner_pkg/pointcloud_processor.h"
#include "avoid_planner_pkg/utils.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <limits>

namespace mid360_avoidance {

/**
 * 构造函数
 * @param nh ros句柄
 * @brief 构造函数确定，参数导入 
 */
PointcloudProcessor::PointcloudProcessor(ros::NodeHandle& nh) : 
    nh_(nh), tf_listener_(tf_buffer_), is_updated_(false) {
    
    //导入参数

    //加载参数==============
    nh_.param<double>("max_sensor_range", max_sensor_range_, 50.0);//最大探测范围
    nh_.param<double>("min_sensor_range", min_sensor_range_, 0.5);//最小探测范围
    nh_.param<double>("voxel_grid_size", voxel_grid_size_, 0.1);
    nh_.param<int>("statistical_filter_mean_k", statistical_filter_mean_k_, 10);//统计滤波的邻域点数量
    nh_.param<double>("statistical_filter_std_dev", statistical_filter_std_dev_, 0.1);//统计滤波标准差阈值
    nh_.param<std::string>("lidar_frame_id", lidar_frame_id_, "livox_frame");
    nh_.param<std::string>("body_frame_id", body_frame_id_, "base_link");
    
    // 订阅点云话题
    pointcloud_sub_ = nh_.subscribe("/livox/lidar", 10, 
                                   &PointcloudProcessor::pointcloudCallback, this);
    
    ROS_INFO("PointcloudProcessor initialized with parameters:");
    ROS_INFO("  max_sensor_range: %.2f m", max_sensor_range_);
    ROS_INFO("  min_sensor_range: %.2f m", min_sensor_range_);
    ROS_INFO("  voxel_grid_size: %.2f m", voxel_grid_size_);
    ROS_INFO("  lidar_frame_id: %s", lidar_frame_id_.c_str());
    ROS_INFO("  body_frame_id: %s", body_frame_id_.c_str());
}

} // namespace mid360_avoidance