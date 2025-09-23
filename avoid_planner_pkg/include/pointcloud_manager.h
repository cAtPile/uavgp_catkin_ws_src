#ifndef POINTCLOUD_MANAGER_H
#define POINTCLOUD_MANAGER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>
#include <mutex>
#include <vector>

namespace avoidance {

/**
 * @brief 点云管理器类，负责订阅点云数据并转换为路径规划器可用格式
 */
class PointCloudManager {
public:
    /**
     * @brief 构造函数
     * @param nh 节点句柄
     * @param frame_id 目标坐标系ID
     */
    PointCloudManager(ros::NodeHandle& nh, const std::string& frame_id = "local_origin");

    /**
     * @brief 析构函数
     */
    ~PointCloudManager() = default;

    /**
     * @brief 初始化点云订阅器
     * @param topic 点云订阅话题
     */
    void initializeSubscriber(const std::string& topic);

    /**
     * @brief 获取处理后的点云数据
     * @return 转换到目标坐标系的点云
     */
    pcl::PointCloud<pcl::PointXYZI> getProcessedPointcloud() const;

    /**
     * @brief 检查是否有新的点云数据
     * @return 有新数据返回true，否则返回false
     */
    bool hasNewData() const;

    /**
     * @brief 重置新数据标志
     */
    void resetNewDataFlag();

private:
    /**
     * @brief 点云回调函数，处理原始点云并转换坐标系
     * @param msg 原始点云消息
     */
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

    /**
     * @brief 将点云转换到目标坐标系
     * @param in_cloud 输入点云
     * @param out_cloud 转换后的点云
     * @param target_frame 目标坐标系
     * @return 转换成功返回true，否则返回false
     */
    bool transformPointcloud(const pcl::PointCloud<pcl::PointXYZ>& in_cloud,
                             pcl::PointCloud<pcl::PointXYZI>& out_cloud,
                             const std::string& target_frame);

    ros::NodeHandle nh_;                  ///< 节点句柄
    ros::Subscriber pointcloud_sub_;      ///< 点云订阅器
    tf::TransformListener tf_listener_;   ///< TF监听器
    std::string target_frame_;            ///< 目标坐标系

    mutable std::mutex cloud_mutex_;                  ///< 点云数据互斥锁
    pcl::PointCloud<pcl::PointXYZI> processed_cloud_; ///< 处理后的点云
    bool has_new_data_ = false;                       ///< 新数据标志
};

} // namespace avoidance

#endif // POINTCLOUD_MANAGER_H