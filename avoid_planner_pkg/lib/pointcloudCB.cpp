/**
 * @file pointcloudCB.cpp
 * @brief 点云信息回调
 * @details 过滤/坐标系转换/生成dismap
 * @author apoc
 * @date 2025/10/16
 */
#include"avoid_planner_pkg/avoid_planner.h"

/**
 * @brief pointcloudCB
 * @details
 * @param msg
 * @see filterPC()
 * @see tfBodyFrame()
 * @see generatePFdismap()
 */
void AvoidPlanner::pointcloudCB(const sensor_msgs::PointCloud2::ConstPtr& msg){
    
    //信息
    ROS_INFO_THROTTLE(1.0, "resive PointsCloud,wide=%d,high=%d,pointsNum=%d",
        msg->width, msg->height, msg->width * msg->height);
    
    // 将ROS点云消息转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ> raw_cloud;
    pcl::fromROSMsg(*msg, raw_cloud);
    
    //是否为空殿宇
    if (raw_cloud.empty()) {
        ROS_WARN_THROTTLE(1.0, "Received empty pointcloud");
        return;
    }
    
    // 过滤点云
    pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
    filterPC(raw_cloud, filtered_cloud);
    
    //过滤后是否为空
    if (filtered_cloud.empty()) {
        ROS_WARN_THROTTLE(1.0, "Filtered pointcloud is empty");
        return;
    }
    
    // 转换到机体坐标系
    pcl::PointCloud<pcl::PointXYZ> body_cloud;
    if (!tfBodyFrame(filtered_cloud, body_cloud)) {
        ROS_WARN_THROTTLE(1.0, "Failed to transform pointcloud to body frame");
        return;
    }
    
    //机体坐标系点云是否为空
    if (body_cloud.empty()) {
        ROS_WARN_THROTTLE(1.0, "Pointcloud is empty after transformation");
        return;
    }
    
    // 生成极坐标直方图
    generatePFdismap(body_cloud);
    
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