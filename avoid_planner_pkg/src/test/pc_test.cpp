#include <ros/ros.h>
#include <opencv2/opencv.hpp>  // OpenCV库，用于生成与保存直方图图像
#include <opencv2/core/eigen.hpp>  // OpenCV与Eigen数据转换
#include "avoid_planner_pkg/pointcloud_processor.h"  // 引入点云处理器类
#include <chrono>  // 用于控制图像保存频率
#include <iostream>

// 全局变量：点云处理器对象指针（避免函数传参繁琐）
mid360_avoidance::PointcloudProcessor* processor_ptr = nullptr;

/**
 * @brief 直方图可视化核心函数：将极坐标直方图转换为OpenCV图像并保存
 * @param save_path 图像保存路径（默认保存在当前工作目录）
 * @return 布尔值：true表示图像生成并保存成功，false表示失败
 */
bool visualizeHistogram(const std::string& save_path = "./histogram_visualization.png") {
    if (processor_ptr == nullptr) {
        ROS_ERROR("PointcloudProcessor pointer is null!");
        return false;
    }

    // 1. 获取最新的极坐标直方图（只读，线程安全）
    const mid360_avoidance::PolarHistogram& hist = processor_ptr->getHistogram();

    // 检查直方图数据有效性（避免空数据导致崩溃）
    if (hist.data.empty() || hist.num_azimuth_bins == 0 || hist.num_elevation_bins == 0) {
        ROS_WARN("Histogram data is invalid (empty or zero bins)!");
        return false;
    }

    // 2. 配置图像参数（根据直方图维度设置图像尺寸，添加边缘留白）
    const int bin_pixel_size = 5;  // 每个直方图网格对应图像的像素大小（控制图像清晰度）
    const int margin = 20;          // 图像边缘留白（用于显示坐标轴标签）
    const int img_width = hist.num_azimuth_bins * bin_pixel_size + 2 * margin;  // 图像宽度
    const int img_height = hist.num_elevation_bins * bin_pixel_size + 2 * margin;  // 图像高度

    // 创建空白图像（3通道RGB，初始为白色背景）
    cv::Mat hist_image(img_height, img_width, CV_8UC3, cv::Scalar(255, 255, 255));

    // 3. 定义距离映射参数（将障碍物距离转换为颜色：近→红，远→绿，无障碍物→蓝）
    const double min_dist = processor_ptr->getMinSensorRange();  // 传感器最小探测距离（0.5m）
    const double max_dist = processor_ptr->getMaxSensorRange();  // 传感器最大探测距离（50m）
    const double dist_range = max_dist - min_dist;               // 距离范围（用于归一化）

    // 4. 遍历直方图每个网格，根据障碍物距离绘制颜色块
    for (size_t az = 0; az < hist.num_azimuth_bins; ++az) {  // 遍历方位角维度（x轴）
        for (size_t el = 0; el < hist.num_elevation_bins; ++el) {  // 遍历仰角维度（y轴）
            double obstacle_dist = hist.data[az][el];  // 当前网格的障碍物距离

            // 定义当前网格在图像中的像素位置（添加边缘留白偏移）
            int x_start = margin + az * bin_pixel_size;
            int y_start = margin + el * bin_pixel_size;
            int x_end = x_start + bin_pixel_size;
            int y_end = y_start + bin_pixel_size;

            // 绘制矩形的像素坐标（确保不超出图像边界）
            cv::Rect bin_rect(
                cv::Point(std::max(x_start, 0), std::max(y_start, 0)),
                cv::Point(std::min(x_end, img_width - 1), std::min(y_end, img_height - 1))
            );

            // 根据障碍物距离确定颜色：
            cv::Scalar bin_color;
            if (obstacle_dist == INFINITY) {
                // 无障碍物（距离为无穷大）→ 蓝色（RGB：0,0,255）
                bin_color = cv::Scalar(255, 0, 0);
            } else {
                // 有障碍物：距离归一化到[0,1]，近→红（255,0,0），远→绿（0,255,0）
                double norm_dist = std::min(std::max((obstacle_dist - min_dist) / dist_range, 0.0), 1.0);
                int red = static_cast<int>((1 - norm_dist) * 255);  // 距离越近，红色越浓
                int green = static_cast<int>(norm_dist * 255);      // 距离越远，绿色越浓
                bin_color = cv::Scalar(0, green, red);  // OpenCV颜色通道为BGR，故顺序为（0, G, R）
            }

            // 在图像上绘制当前网格的颜色块
            cv::rectangle(hist_image, bin_rect, bin_color, -1);  // -1表示填充矩形
        }
    }

    // 5. 添加图像标签（坐标轴说明、距离图例）
    // 5.1 绘制坐标轴标签
    std::string az_label = "Azimuth (0 to 360 deg)";  // 方位角标签（0~360度）
    std::string el_label = "Elevation (-90 to 90 deg)";  // 仰角标签（-90~90度）
    cv::putText(hist_image, az_label, 
                cv::Point(margin, img_height - 5),  // 位置：底部留白处
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);  // 黑色字体，字号0.6
    cv::putText(hist_image, el_label, 
                cv::Point(5, margin + img_height / 2),  // 位置：左侧留白处（垂直居中）
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1, 
                cv::LINE_AA, true);  // 垂直绘制文字

    // 5.2 绘制距离图例（说明颜色与距离的对应关系）
    const int legend_width = 150;  // 图例宽度
    const int legend_height = 80;  // 图例高度
    cv::Rect legend_rect(cv::Point(img_width - margin - legend_width, margin), 
                        cv::Size(legend_width, legend_height));
    cv::rectangle(hist_image, legend_rect, cv::Scalar(0, 0, 0), 1);  // 图例边框（黑色）
    // 图例文字说明
    cv::putText(hist_image, "Obstacle Distance", 
                cv::Point(img_width - margin - legend_width + 10, margin + 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
    cv::putText(hist_image, "Near (Red)  0.5m", 
                cv::Point(img_width - margin - legend_width + 10, margin + 40),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);  // 红色文字表示近距离
    cv::putText(hist_image, "Far (Green) 50m", 
                cv::Point(img_width - margin - legend_width + 10, margin + 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);  // 绿色文字表示远距离
    cv::putText(hist_image, "None (Blue)", 
                cv::Point(img_width - margin - legend_width + 10, margin + 80),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0), 1);  // 蓝色文字表示无障碍物

    // 6. 保存图像到指定路径
    if (!cv::imwrite(save_path, hist_image)) {
        ROS_ERROR("Failed to save histogram image to path: %s", save_path.c_str());
        return false;
    }

    ROS_INFO("Histogram image saved successfully! Path: %s", save_path.c_str());
    return true;
}

/**
 * @brief 主函数：初始化ROS节点、点云处理器，循环检测直方图更新并生成图像
 */
int main(int argc, char** argv) {
    // 1. 初始化ROS节点（节点名：mid360_histogram_test）
    ros::init(argc, argv, "mid360_histogram_test");
    ros::NodeHandle nh;  // 创建ROS节点句柄

    // 2. 初始化点云处理器（传入节点句柄，自动加载参数并订阅点云话题）
    mid360_avoidance::PointcloudProcessor processor(nh);
    processor_ptr = &processor;  // 赋值给全局指针，供可视化函数使用

    // 3. 配置图像保存参数
    std::string save_path = "./mid360_histogram.png";  // 默认保存路径
    double save_interval = 2.0;  // 图像保存间隔（2秒/张，避免频繁写入）
    auto last_save_time = std::chrono::steady_clock::now();  // 上次保存时间

    // 4. 循环检测直方图更新并生成图像（ROS自旋+定时保存）
    ROS_INFO("Start histogram visualization test. Press Ctrl+C to exit.");
    ros::Rate rate(10);  // 循环频率：10Hz（确保实时性）
    while (ros::ok()) {
        // 4.1 检查直方图是否更新（有新的点云数据处理完成）
        if (processor.isUpdated()) {
            // 4.2 控制保存频率（避免同一批数据重复保存）
            auto current_time = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(current_time - last_save_time).count();
            if (elapsed >= save_interval) {
                // 生成并保存直方图图像
                visualizeHistogram(save_path);
                last_save_time = current_time;  // 更新上次保存时间
                processor.resetUpdatedFlag();  // 复位更新标志，等待下一批数据
            }
        }

        // 4.3 处理ROS回调（点云订阅回调）
        ros::spinOnce();
        rate.sleep();  // 按设定频率休眠
    }

    // 5. 程序退出
    ROS_INFO("Histogram test program exited normally.");
    return 0;
}