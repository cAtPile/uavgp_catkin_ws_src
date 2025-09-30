#include "avoid_planner_pkg/pointcloud_processor.h"
#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <thread>
#include <chrono>

// 全局可视化器指针
pcl::visualization::PCLVisualizer::Ptr viewer;

// 可视化点云和直方图信息
void visualizeData(const mid360_avoidance::PointcloudProcessor& processor) {
    if (!viewer) {
        viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Pointcloud Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(1.0);
    }

    // 获取直方图数据
    const mid360_avoidance::PolarHistogram& histogram = processor.getHistogram();
    
    // 输出直方图基本信息
    ROS_INFO_STREAM("Histogram info - Azimuth bins: " << histogram.num_azimuth_bins 
                  << ", Elevation bins: " << histogram.num_elevation_bins);

    // 统计有障碍物的网格数量
    int obstacle_count = 0;
    double min_distance = std::numeric_limits<double>::infinity();
    double max_distance = 0.0;
    
    for (const auto& az_bin : histogram.data) {
        for (double dist : az_bin) {
            if (dist != std::numeric_limits<double>::infinity()) {
                obstacle_count++;
                min_distance = std::min(min_distance, dist);
                max_distance = std::max(max_distance, dist);
            }
        }
    }
    
    ROS_INFO_STREAM("Obstacle grid count: " << obstacle_count 
                  << ", Min distance: " << min_distance 
                  << "m, Max distance: " << max_distance << "m");

    // 如果有新数据则更新可视化
    if (processor.isUpdated()) {
        viewer->removeAllPointClouds();
        
        // 这里可以添加点云可视化代码
        // 注意：需要修改PointcloudProcessor以提供原始点云数据访问
        // 或在测试中单独订阅点云话题
        
        processor.resetUpdatedFlag();
    }

    // 刷新可视化窗口
    if (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}

int main(int argc, char**argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "pointcloud_processor_test");
    ros::NodeHandle nh;
    
    ROS_INFO("Starting Pointcloud Processor Test...");
    
    // 创建PointcloudProcessor实例
    mid360_avoidance::PointcloudProcessor processor(nh);
    
    // 启动可视化线程
    std::thread visualization_thread([&processor]() {
        while (ros::ok() && !viewer->wasStopped()) {
            visualizeData(processor);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });
    
    // 处理ROS回调
    ros::spin();
    
    // 等待可视化线程结束
    if (visualization_thread.joinable()) {
        visualization_thread.join();
    }
    
    ROS_INFO("Test finished.");
    return 0;
}
