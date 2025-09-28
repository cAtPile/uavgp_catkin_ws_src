// 1. 所有 #include 指令必须放在文件最顶部，顺序：系统库 → 第三方库 → 自定义库
#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <limits>
#include <string>
#include <mutex>

// Eigen 头文件（必须在 OpenCV 之前包含，避免 eigen.hpp 找不到 Eigen 命名空间）
#include <Eigen/Core>
#include <Eigen/Geometry>

// OpenCV 头文件（用于直方图可视化）
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>  // Eigen 与 OpenCV 数据转换

// TF2 头文件
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// 自定义库头文件
#include "avoid_planner_pkg/pointcloud_processor.h"
#include "avoid_planner_pkg/utils.h"

// 命名空间（与 PointcloudProcessor 保持一致）
namespace mid360_avoidance {

/**
 * @brief 可视化极坐标直方图并保存为图片
 * @param processor_ptr PointcloudProcessor 实例指针（用于获取传感器量程）
 * @param save_path 图片保存路径（如 "/home/a/histogram.png"）
 * @return 成功返回 true，失败返回 false
 */
bool visualizeHistogram(const PointcloudProcessor::Ptr& processor_ptr, const std::string& save_path) {
    if (!processor_ptr) {
        ROS_ERROR("visualizeHistogram: PointcloudProcessor pointer is null!");
        return false;
    }

    // 1. 获取直方图数据（加锁确保线程安全）
    const PolarHistogram& hist = processor_ptr->getHistogram();
    if (hist.data.empty() || hist.data[0].empty()) {
        ROS_WARN("visualizeHistogram: Histogram data is empty!");
        return false;
    }

    // 2. 获取传感器量程（直接从 processor 的成员变量获取，避免未定义的 get 函数）
    // 注意：需确保 PointcloudProcessor 类中这两个成员变量是 public，或提供正确的 get 函数
    const double min_dist = processor_ptr->min_sensor_range_;  // 若为 private，需替换为 getMinSensorRange()
    const double max_dist = processor_ptr->max_sensor_range_;  // 若为 private，需替换为 getMaxSensorRange()
    const double dist_range = max_dist - min_dist;

    if (dist_range <= 0) {
        ROS_ERROR("visualizeHistogram: Invalid sensor range (min >= max)!");
        return false;
    }

    // 3. 配置图片参数（可根据需要调整分辨率）
    const int img_width = hist.num_azimuth_bins;    // 方位角维度对应图片宽度
    const int img_height = hist.num_elevation_bins; // 仰角维度对应图片高度
    cv::Mat hist_image(img_height, img_width, CV_8UC3, cv::Scalar(255, 255, 255)); // 白色背景

    // 4. 遍历直方图，绘制每个bin的颜色（距离越近，颜色越红；无障碍物为白色）
    for (size_t az = 0; az < hist.num_azimuth_bins; ++az) {
        for (size_t el = 0; el < hist.num_elevation_bins; ++el) {
            double dist = hist.data[az][el];
            cv::Vec3b color;

            if (dist == std::numeric_limits<double>::infinity()) {
                // 无障碍物：保持白色
                color = cv::Vec3b(255, 255, 255);
            } else {
                // 有障碍物：距离映射为颜色（0→红，dist_range→蓝）
                double normalized_dist = (dist - min_dist) / dist_range;
                uchar red = static_cast<uchar>((1 - normalized_dist) * 255);  // 近→红
                uchar blue = static_cast<uchar>(normalized_dist * 255);       // 远→蓝
                color = cv::Vec3b(blue, 0, red);  // OpenCV 颜色顺序：BGR
            }

            // 绘制像素（注意：OpenCV 坐标 (x,y) 对应 (az, el)，需确认是否翻转）
            hist_image.at<cv::Vec3b>(el, az) = color;
        }
    }

    // 5. 保存图片
    if (!cv::imwrite(save_path, hist_image)) {
        ROS_ERROR("visualizeHistogram: Failed to save image to path: %s", save_path.c_str());
        return false;
    }

    ROS_INFO("visualizeHistogram: Image saved successfully! Path: %s", save_path.c_str());
    return true;
}

} // namespace mid360_avoidance

/**
 * @brief 主函数：测试 PointcloudProcessor 与直方图可视化
 */
int main(int argc, char** argv) {
    // 1. 初始化 ROS 节点
    ros::init(argc, argv, "pcp_test_node");
    ros::NodeHandle nh("~");  // 私有节点句柄，用于加载参数
    ROS_INFO("pcp_test_node started!");

    try {
        // 2. 创建 PointcloudProcessor 实例（智能指针，避免内存泄漏）
        mid360_avoidance::PointcloudProcessor::Ptr processor_ptr(
            new mid360_avoidance::PointcloudProcessor(nh)
        );
        ROS_INFO("PointcloudProcessor initialized successfully!");

        // 3. 配置参数（可通过 ROS 参数服务器设置，默认保存到用户目录）
        std::string save_path;
        nh.param<std::string>("histogram_save_path", save_path, "/home/a/histogram.png");
        ROS_INFO("Histogram will be saved to: %s", save_path.c_str());

        // 4. 等待点云数据更新（超时30秒）
        ROS_INFO("Waiting for pointcloud data... (timeout: 30s)");
        const ros::Time start_time = ros::Time::now();
        bool received_data = false;

        while (ros::ok() && (ros::Time::now() - start_time).toSec() < 30.0) {
            if (processor_ptr->isUpdated()) {
                received_data = true;
                ROS_INFO("Received valid pointcloud data, generating histogram...");

                // 5. 可视化并保存直方图
                mid360_avoidance::visualizeHistogram(processor_ptr, save_path);

                // 6. 重置更新标志，避免重复处理
                processor_ptr->resetUpdatedFlag();
                break;
            }
            ros::spinOnce();  // 处理回调函数
            ros::Duration(0.1).sleep();  // 10Hz 循环
        }

        // 处理超时情况
        if (!received_data) {
            ROS_WARN("pcp_test_node: Timeout waiting for pointcloud data (30s)!");
        }

        // 7. 主循环：持续监控点云（按 Ctrl+C 退出）
        ROS_INFO("Entering main loop. Press Ctrl+C to exit.");
        ros::Rate rate(10);  // 10Hz
        while (ros::ok()) {
            if (processor_ptr->isUpdated()) {
                ROS_INFO("Pointcloud updated (histogram ready)");
                processor_ptr->resetUpdatedFlag();
                // 可选：每次更新都保存直方图（取消注释下面一行）
                // mid360_avoidance::visualizeHistogram(processor_ptr, save_path);
            }
            ros::spinOnce();
            rate.sleep();
        }

    } catch (const std::exception& e) {
        // 捕获异常，打印错误信息
        ROS_ERROR("pcp_test_node: Exception occurred: %s", e.what());
        return 1;
    }

    // 正常退出
    ROS_INFO("pcp_test_node exited successfully!");
    return 0;
}