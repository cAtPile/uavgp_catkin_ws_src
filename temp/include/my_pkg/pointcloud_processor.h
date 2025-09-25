//点云处理模块声明


#ifndef MID360_AVOIDANCE_POINTCLOUD_PROCESSOR_H
#define MID360_AVOIDANCE_POINTCLOUD_PROCESSOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <mutex>

namespace mid360_avoidance {

/**
 * @struct PolarHistogram
 * @brief 极坐标直方图结构，存储每个角度网格中的障碍物距离
 */
struct PolarHistogram {
    // 角度分辨率配置
    double azimuth_resolution;  // 方位角分辨率(弧度)
    double elevation_resolution; // 仰角分辨率(弧度)
    
    // 角度范围 (Mid360参数: 方位角360°, 仰角±15°)
    double min_azimuth;         // 最小方位角(弧度)
    double max_azimuth;         // 最大方位角(弧度)
    double min_elevation;       // 最小仰角(弧度)
    double max_elevation;       // 最大仰角(弧度)
    
    // 网格尺寸
    size_t num_azimuth_bins;    // 方位角网格数量
    size_t num_elevation_bins;  // 仰角网格数量
    
    // 直方图数据: [azimuth][elevation] = 障碍物距离(m)
    std::vector<std::vector<double>> data;
    
    // 时间戳
    ros::Time timestamp;
    
    // 构造函数
    PolarHistogram() : 
        azimuth_resolution(0.01745),  // 约1°(弧度)
        elevation_resolution(0.0873), // 约5°(弧度)
        min_azimuth(-M_PI),
        max_azimuth(M_PI),
        min_elevation(-0.2618),      // -15°(弧度)
        max_elevation(0.2618),       // +15°(弧度)
        num_azimuth_bins(360),
        num_elevation_bins(7),
        data(360, std::vector<double>(7, INFINITY)) {}
};

/**
 * @class PointcloudProcessor
 * @brief 点云处理器类，负责处理Mid360激光雷达数据并生成障碍物直方图
 */
class PointcloudProcessor {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    explicit PointcloudProcessor(ros::NodeHandle& nh);
    
    /**
     * @brief 析构函数
     */
    ~PointcloudProcessor() = default;
    
    /**
     * @brief 获取处理后的极坐标直方图
     * @return 极坐标直方图引用
     */
    const PolarHistogram& getHistogram() const;
    
    /**
     * @brief 检查是否有新的直方图数据
     * @return 若有新数据则返回true，否则返回false
     */
    bool isUpdated() const;
    
    /**
     * @brief 重置更新标志
     */
    void resetUpdatedFlag();

private:
    /**
     * @brief 点云消息回调函数
     * @param msg 点云消息指针
     */
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    
    /**
     * @brief 过滤原始点云
     * @param input 输入点云
     * @param output 输出过滤后的点云
     */
    void filterPointcloud(const pcl::PointCloud<pcl::PointXYZ>& input, 
                         pcl::PointCloud<pcl::PointXYZ>& output);
    
    /**
     * @brief 将点云从雷达坐标系转换到机体坐标系
     * @param input 输入点云(雷达坐标系)
     * @param output 输出点云(机体坐标系)
     * @return 转换成功返回true，否则返回false
     */
    bool transformToBodyFrame(const pcl::PointCloud<pcl::PointXYZ>& input, 
                             pcl::PointCloud<pcl::PointXYZ>& output);
    
    /**
     * @brief 生成极坐标直方图
     * @param cloud 机体坐标系下的点云
     */
    void generatePolarHistogram(const pcl::PointCloud<pcl::PointXYZ>& cloud);
    
    /**
     * @brief 将笛卡尔坐标点转换为极坐标并更新直方图
     * @param x X坐标(机体坐标系)
     * @param y Y坐标(机体坐标系)
     * @param z Z坐标(机体坐标系)
     */
    void updateHistogramFromPoint(double x, double y, double z);
    
    /**
     * @brief 将角度值映射到直方图网格索引
     * @param angle 角度值(弧度)
     * @param min_angle 最小角度(弧度)
     * @param max_angle 最大角度(弧度)
     * @param num_bins 网格数量
     * @return 网格索引，若超出范围返回-1
     */
    int angleToBinIndex(double angle, double min_angle, double max_angle, size_t num_bins);

    // ROS相关成员
    ros::NodeHandle nh_;                  // ROS节点句柄
    ros::Subscriber pointcloud_sub_;      // 点云订阅者
    tf2_ros::Buffer tf_buffer_;           // TF变换缓存
    tf2_ros::TransformListener tf_listener_; // TF变换监听器
    
    // 点云处理参数
    double max_sensor_range_;             // 最大感知距离(m)
    double min_sensor_range_;             // 最小感知距离(m)
    double voxel_grid_size_;              // 体素滤波分辨率(m)
    int statistical_filter_mean_k_;       // 统计滤波邻域点数量
    double statistical_filter_std_dev_;   // 统计滤波标准差阈值
    std::string lidar_frame_id_;          // 激光雷达坐标系ID
    std::string body_frame_id_;           // 机体坐标系ID
    
    // 直方图数据
    PolarHistogram histogram_;            // 极坐标直方图
    mutable std::mutex histogram_mutex_;  // 直方图数据锁
    bool is_updated_;                     // 更新标志
    mutable std::mutex updated_mutex_;    // 更新标志锁
};

} // namespace mid360_avoidance

#endif // MID360_AVOIDANCE_POINTCLOUD_PROCESSOR_H
