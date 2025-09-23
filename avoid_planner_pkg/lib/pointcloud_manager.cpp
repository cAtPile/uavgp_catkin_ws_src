#include "pointcloud_manager.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_datatypes.h>
#include <ros/console.h>

namespace avoidance {

PointCloudManager::PointCloudManager(ros::NodeHandle& nh, const std::string& frame_id)
    : nh_(nh), target_frame_(frame_id) {
    // 初始化TF监听器，设置缓存时间
    tf_listener_.setExtrapolationLimit(ros::Duration(0.1));
    ROS_INFO_STREAM("PointCloudManager initialized with target frame: " << target_frame_);
}

void PointCloudManager::initializeSubscriber(const std::string& topic) {
    if (pointcloud_sub_) {
        ROS_WARN("Point cloud subscriber already initialized, reinitializing...");
        pointcloud_sub_.shutdown();
    }

    // 订阅点云话题
    pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        topic, 1, &PointCloudManager::pointCloudCallback, this);
    ROS_INFO_STREAM("Subscribed to point cloud topic: " << topic);
}

pcl::PointCloud<pcl::PointXYZI> PointCloudManager::getProcessedPointcloud() const {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    return processed_cloud_;
}

bool PointCloudManager::hasNewData() const {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    return has_new_data_;
}

void PointCloudManager::resetNewDataFlag() {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    has_new_data_ = false;
}

void PointCloudManager::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // 将ROS点云消息转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ> raw_cloud;
    pcl::fromROSMsg(*msg, raw_cloud);

    if (raw_cloud.empty()) {
        ROS_WARN_THROTTLE(1, "Received empty point cloud");
        return;
    }

    // 转换点云到目标坐标系
    pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
    if (!transformPointcloud(raw_cloud, transformed_cloud, target_frame_)) {
        ROS_WARN_THROTTLE(1, "Failed to transform point cloud to target frame: %s", 
                         target_frame_.c_str());
        return;
    }

    // 点云预处理：下采样（减少数据量）
    pcl::PointCloud<pcl::PointXYZI> filtered_cloud;
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(transformed_cloud.makeShared());
    voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);  // 10cm体素网格
    voxel_filter.filter(filtered_cloud);

    // 点云预处理：范围过滤（只保留一定范围内的点）
    pcl::PassThrough<pcl::PointXYZI> pass_filter;
    pass_filter.setInputCloud(filtered_cloud.makeShared());
    pass_filter.setFilterFieldName("x");
    pass_filter.setFilterLimits(-10.0f, 10.0f);  // x范围：±10米
    pass_filter.filter(filtered_cloud);

    pass_filter.setFilterFieldName("y");
    pass_filter.setFilterLimits(-10.0f, 10.0f);  // y范围：±10米
    pass_filter.filter(filtered_cloud);

    pass_filter.setFilterFieldName("z");
    pass_filter.setFilterLimits(-2.0f, 5.0f);    // z范围：-2米到5米
    pass_filter.filter(filtered_cloud);

    if (filtered_cloud.empty()) {
        ROS_DEBUG_THROTTLE(1, "No points remaining after filtering");
        return;
    }

    // 保存处理后的点云并标记新数据
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    processed_cloud_ = filtered_cloud;
    has_new_data_ = true;
}

bool PointCloudManager::transformPointcloud(const pcl::PointCloud<pcl::PointXYZ>& in_cloud,
                                           pcl::PointCloud<pcl::PointXYZI>& out_cloud,
                                           const std::string& target_frame) {
    if (in_cloud.empty()) {
        ROS_WARN("Input cloud is empty");
        return false;
    }

    // 获取点云原始坐标系
    std::string source_frame = in_cloud.header.frame_id;
    if (source_frame.empty()) {
        ROS_ERROR("Point cloud has no frame ID");
        return false;
    }

    // 如果源坐标系与目标坐标系相同，直接转换格式
    if (source_frame == target_frame) {
        out_cloud.resize(in_cloud.size());
        for (size_t i = 0; i < in_cloud.size(); ++i) {
            out_cloud[i].x = in_cloud[i].x;
            out_cloud[i].y = in_cloud[i].y;
            out_cloud[i].z = in_cloud[i].z;
            out_cloud[i].intensity = 1.0f;  // 默认强度值
        }
        out_cloud.header.frame_id = target_frame;
        return true;
    }

    // 查找坐标变换
    tf::StampedTransform transform;
    try {
        // 等待变换可用（最多等待0.1秒）
        tf_listener_.waitForTransform(target_frame, source_frame,
                                     ros::Time(in_cloud.header.stamp),
                                     ros::Duration(0.1));
        tf_listener_.lookupTransform(target_frame, source_frame,
                                    ros::Time(in_cloud.header.stamp),
                                    transform);
    }
    catch (tf::TransformException& ex) {
        ROS_WARN_THROTTLE(1, "Transform exception: %s", ex.what());
        return false;
    }

    // 将TF变换转换为Eigen矩阵
    Eigen::Affine3d eigen_transform;
    tf::transformTFToEigen(transform, eigen_transform);

    // 应用变换到每个点
    out_cloud.reserve(in_cloud.size());
    for (const auto& point : in_cloud) {
        // 转换点到Eigen向量
        Eigen::Vector3d eigen_point(point.x, point.y, point.z);
        
        // 应用坐标变换
        Eigen::Vector3d transformed_point = eigen_transform * eigen_point;

        // 保存到输出点云（添加强度值）
        pcl::PointXYZI out_point;
        out_point.x = static_cast<float>(transformed_point.x());
        out_point.y = static_cast<float>(transformed_point.y());
        out_point.z = static_cast<float>(transformed_point.z());
        out_point.intensity = 1.0f;  // 可根据实际需求设置强度值
        out_cloud.push_back(out_point);
    }

    out_cloud.header.frame_id = target_frame;
    return true;
}

} // namespace avoidance
