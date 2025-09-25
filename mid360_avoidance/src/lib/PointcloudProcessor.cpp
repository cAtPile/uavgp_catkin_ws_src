#include "mid360_avoidance/pointcloud_processor.h"
#include "mid360_avoidance/utils.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <limits>

namespace mid360_avoidance {

//构造函数
PointcloudProcessor::PointcloudProcessor(ros::NodeHandle& nh) : 
    nh_(nh), tf_listener_(tf_buffer_), is_updated_(false) {
    
    //加载参数==============
    nh_.param<double>("max_sensor_range", max_sensor_range_, 50.0);//最大探测范围
    nh_.param<double>("min_sensor_range", min_sensor_range_, 0.5);//最小探测范围
    nh_.param<double>("voxel_grid_size", voxel_grid_size_, 0.1);
    nh_.param<int>("statistical_filter_mean_k", statistical_filter_mean_k_, 10);
    nh_.param<double>("statistical_filter_std_dev", statistical_filter_std_dev_, 0.1);
    nh_.param<std::string>("lidar_frame_id", lidar_frame_id_, "mid360_link");
    nh_.param<std::string>("body_frame_id", body_frame_id_, "base_link");
    
    // 订阅点云话题
    pointcloud_sub_ = nh_.subscribe("/mid360/points", 10, //WRAN
                                   &PointcloudProcessor::pointcloudCallback, this);
    
    ROS_INFO("PointcloudProcessor initialized with parameters:");
    ROS_INFO("  max_sensor_range: %.2f m", max_sensor_range_);
    ROS_INFO("  min_sensor_range: %.2f m", min_sensor_range_);
    ROS_INFO("  voxel_grid_size: %.2f m", voxel_grid_size_);
    ROS_INFO("  lidar_frame_id: %s", lidar_frame_id_.c_str());
    ROS_INFO("  body_frame_id: %s", body_frame_id_.c_str());
}

const PolarHistogram& PointcloudProcessor::getHistogram() const {
    std::lock_guard<std::mutex> lock(histogram_mutex_);
    return histogram_;
}

bool PointcloudProcessor::isUpdated() const {
    std::lock_guard<std::mutex> lock(updated_mutex_);
    return is_updated_;
}

void PointcloudProcessor::resetUpdatedFlag() {
    std::lock_guard<std::mutex> lock(updated_mutex_);
    is_updated_ = false;
}

void PointcloudProcessor::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // 将ROS点云消息转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ> raw_cloud;
    pcl::fromROSMsg(*msg, raw_cloud);
    
    if (raw_cloud.empty()) {
        ROS_WARN_THROTTLE(1.0, "Received empty pointcloud");
        return;
    }
    
    // 过滤点云
    pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
    filterPointcloud(raw_cloud, filtered_cloud);
    
    if (filtered_cloud.empty()) {
        ROS_DEBUG_THROTTLE(1.0, "Filtered pointcloud is empty");
        return;
    }
    
    // 转换到机体坐标系
    pcl::PointCloud<pcl::PointXYZ> body_cloud;
    if (!transformToBodyFrame(filtered_cloud, body_cloud)) {
        ROS_WARN_THROTTLE(1.0, "Failed to transform pointcloud to body frame");
        return;
    }
    
    if (body_cloud.empty()) {
        ROS_DEBUG_THROTTLE(1.0, "Pointcloud is empty after transformation");
        return;
    }
    
    // 生成极坐标直方图
    generatePolarHistogram(body_cloud);
    
    // 更新时间戳和标志
    {
        std::lock_guard<std::mutex> lock(histogram_mutex_);
        histogram_.timestamp = msg->header.stamp;
    }
    
    {
        std::lock_guard<std::mutex> lock(updated_mutex_);
        is_updated_ = true;
    }
}

void PointcloudProcessor::filterPointcloud(const pcl::PointCloud<pcl::PointXYZ>& input, 
                                          pcl::PointCloud<pcl::PointXYZ>& output) {
    pcl::PointCloud<pcl::PointXYZ> temp_cloud;
    
    // 1. 距离滤波 - 移除超出传感器范围的点
    pcl::PassThrough<pcl::PointXYZ> pass_filter;
    pass_filter.setInputCloud(input.makeShared());
    pass_filter.setFilterFieldName("x");
    pass_filter.setFilterLimits(-max_sensor_range_, max_sensor_range_);
    pass_filter.filter(temp_cloud);
    
    if (temp_cloud.empty()) {
        output = temp_cloud;
        return;
    }
    
    // 2. 体素网格滤波 - 下采样
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(temp_cloud.makeShared());
    voxel_filter.setLeafSize(voxel_grid_size_, voxel_grid_size_, voxel_grid_size_);
    voxel_filter.filter(temp_cloud);
    
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

bool PointcloudProcessor::transformToBodyFrame(const pcl::PointCloud<pcl::PointXYZ>& input, 
                                             pcl::PointCloud<pcl::PointXYZ>& output) {
    try {
        // 获取激光雷达到机体坐标系的变换
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            body_frame_id_, lidar_frame_id_, ros::Time(0), ros::Duration(0.1));
        
        // 转换为Eigen变换
        Eigen::Affine3d eigen_transform = tf2::transformToEigen(transform);
        
        // 应用变换到整个点云
        pcl::transformPointCloud(input, output, eigen_transform);
        
        return true;
    }
    catch (tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "Transform exception: %s", ex.what());
        return false;
    }
}

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

} // namespace mid360_avoidance
