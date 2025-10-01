#include "avoid_planner_pkg/pointcloud_processor.h"
#include "avoid_planner_pkg/PolarHistogramMsg.h"  // 假设自定义消息类型
#include <ros/ros.h>

int main(int argc, char**argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "pcp_test_node");
    ros::NodeHandle nh("~");

    try {
        // 创建点云处理器实例
        mid360_avoidance::PointcloudProcessor processor(nh);
        
        // 创建直方图消息发布者
        ros::Publisher hist_pub = nh.advertise<avoid_planner_pkg::PolarHistogramMsg>(
            "/polar_histogram", 10);

        ROS_INFO("Polar histogram publisher node initialized.");

        // 主循环
        ros::Rate rate(20);  // 20Hz循环频率
        while (ros::ok()) {
            // 检查是否有新的直方图数据
            if (processor.isUpdated()) {
                // 获取直方图数据（内部已加锁保护）
                const mid360_avoidance::PolarHistogram& hist = processor.getHistogram();
                
                // 构造ROS消息
                avoid_planner_pkg::PolarHistogramMsg msg;
                
                // 填充消息字段
                msg.header.stamp = hist.timestamp;
                msg.azimuth_resolution = hist.azimuth_resolution;
                msg.elevation_resolution = hist.elevation_resolution;
                msg.min_azimuth = hist.min_azimuth;
                msg.max_azimuth = hist.max_azimuth;
                msg.min_elevation = hist.min_elevation;
                msg.max_elevation = hist.max_elevation;
                msg.max_range = hist.max_range;
                msg.num_azimuth_bins = hist.num_azimuth_bins;
                msg.num_elevation_bins = hist.num_elevation_bins;
                
                // 扁平化二维数据（按方位角-仰角顺序）
                msg.data.clear();
                msg.data.reserve(hist.num_azimuth_bins * hist.num_elevation_bins);
                for (const auto& az_bin : hist.data) {
                    msg.data.insert(msg.data.end(), az_bin.begin(), az_bin.end());
                }
                
                // 发布消息
                hist_pub.publish(msg);
                ROS_DEBUG_THROTTLE(1.0, "Published polar histogram with %zu azimuth bins and %zu elevation bins",
                                  hist.num_azimuth_bins, hist.num_elevation_bins);
                
                // 重置更新标志
                processor.resetUpdatedFlag();
            }

            // 处理回调函数（点云订阅在处理器内部）
            ros::spinOnce();
            rate.sleep();
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Node failed: %s", e.what());
        return 1;
    }

    return 0;
}