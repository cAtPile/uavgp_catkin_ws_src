/**
 * @file pointcloud_callback.cpp
 * @brief 点云回调
 * @author apoc
 * @version 1.0
 * @date 10-5
 */
#include "avoid_planner_pkg/pointcloud_processor.h"
#include "avoid_planner_pkg/utils.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <limits>

namespace avoid_planner {

/**
 * @brief 点云消息回调函数
 * @details 处理接收到的ROS点云消息，执行一系列点云处理操作：
 *          1. 将ROS点云消息转换为PCL点云格式
 *          2. 对原始点云进行过滤处理
 *          3. 将过滤后的点云转换到机体坐标系
 *          4. 根据机体坐标系下的点云生成极坐标直方图
 *          5. 更新时间戳和状态标志
 * 
 * @param[in] msg 传感器点云消息指针，类型为sensor_msgs::PointCloud2::ConstPtr
 * 
 * @note 函数使用ROS_INFO_THROTTLE和ROS_WARN_THROTTLE控制日志输出频率，避免刷屏
 * @note 多处进行空点云检查，保证处理流程的健壮性
 * @note 使用互斥锁保护共享数据的访问
 * 
 * @see filterPointcloud() 用于点云过滤处理
 * @see transformToBodyFrame() 用于点云坐标系转换
 * @see generatePolarHistogram() 用于生成极坐标直方图
 * @see pcl::fromROSMsg() PCL库中ROS消息转PCL点云的函数
 * 
 * @warning 如果点云转换失败，会输出警告并返回
 */
void PointcloudProcessor::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

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
    filterPointcloud(raw_cloud, filtered_cloud);
    
    //过滤后是否为空
    if (filtered_cloud.empty()) {
        ROS_WARN_THROTTLE(1.0, "Filtered pointcloud is empty");
        return;
    }
    
    // 转换到机体坐标系
    pcl::PointCloud<pcl::PointXYZ> body_cloud;
    if (!transformToBodyFrame(filtered_cloud, body_cloud)) {
        ROS_WARN_THROTTLE(1.0, "Failed to transform pointcloud to body frame");
        return;
    }
    
    //机体坐标系点云是否为空
    if (body_cloud.empty()) {
        ROS_WARN_THROTTLE(1.0, "Pointcloud is empty after transformation");
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