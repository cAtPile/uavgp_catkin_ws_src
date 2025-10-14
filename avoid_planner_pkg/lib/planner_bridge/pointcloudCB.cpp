/**
 * @file pointcloudCB.cpp
 * @brief 点云回调
 * @details 接收点云并处理
 * @author apoc
 * @date 2025/10/14
 */
#include"avoid_planner_pkg/planner_bridge.h"

namespace avoid_planner {

// 订阅点云话题
    pointcloud_sub_ = nh_.subscribe(lidar_topic_, 10,
                                   &PlannerBridge::pointcloudCB, this);

/**
 * @brief 点云回调
 * @details 接收点云并处理，填充PF
 * @see pointcloud_processor
 */
void PlannerBridge::pointcloudCB(const sensor_msgs::PointCloud2::ConstPtr& msg){

    //
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
    

}

}  // namespace avoid_planner