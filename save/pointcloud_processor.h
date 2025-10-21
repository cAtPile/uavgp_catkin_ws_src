/**
 * @file pointcloud_processor.h
 * @brief 处理点云
 * @details struct PolarField
 */

#ifndef AVOID_PLANNER_POINTCLOUD_PROCESSOR_H
#define AVOID_PLANNER_POINTCLOUD_PROCESSOR_H

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
#include <cmath> 

namespace avoid_planner {

/**
 * @struct PolarField
 * @brief 极坐标场
 * @details 存储极坐标数据和场
 *          bins需要计算
 */
struct PolarField{

    // 角度分辨率配置
    double azimuth_resolution;  // 方位角分辨率(弧度)
    double elevation_resolution; // 仰角分辨率(弧度)
    
    // 角度范围
    double min_azimuth;         // 最小方位角(弧度)
    double max_azimuth;         // 最大方位角(弧度)
    double min_elevation;       // 最小仰角(弧度)
    double max_elevation;       // 最大仰角(弧度)

    // 传感器参数
    double max_range;   // 最大传感器距离(m)
    double min_range;   // 传感器最小距离(m)
    
    // 网格尺寸
    size_t num_azimuth_bins;    // 方位角网格数量
    size_t num_elevation_bins;  // 仰角网格数量
    
    // 直方图数据: [azimuth][elevation] = 障碍物距离(m)
    std::vector<std::vector<double>> dis_map;

    // 势场数据 [azimuth][elevation] = 力大小（N）
    std::vector<std::vector<double>> pot_map;

    Eigen::Vector3d local_position; // 本地坐标
    Eigen::Vector3d force_vector;   // 合力向量
    
    // 时间戳
    ros::Time timestamp;

    // 无参构造函数
    PolarField():
        azimuth_resolution(0.01745),  // 约1°(弧度)
        elevation_resolution(0.0873), // 约5°(弧度)
        max_range(5.0),  // 初始化最大传感器距离
        min_range(0.5),
        min_azimuth(-M_PI),
        max_azimuth(M_PI),
        min_elevation(-0.122),      // 约-7°(弧度)
        max_elevation(0.984),       // 约+56°(弧度)
        timestamp(ros::Time::now()), // 初始化时间戳为当前时间
        local_position(Eigen::Vector3d::Zero()), // 初始化本地坐标为原点
        force_vector(Eigen::Vector3d::Zero())    // 初始化合力向量为零向量
    {
        // 计算网格数量
        num_azimuth_bins = static_cast<size_t>((max_azimuth - min_azimuth) / azimuth_resolution) + 1;
        num_elevation_bins = static_cast<size_t>((max_elevation - min_elevation) / elevation_resolution) + 1;
        
        // 初始化距离图和势场图
        dis_map.resize(num_azimuth_bins, std::vector<double>(num_elevation_bins, 0.0));
        pot_map.resize(num_azimuth_bins, std::vector<double>(num_elevation_bins, 0.0));
    }

    // 带参数的构造函数
    PolarField(double az_res, double el_res,
               double min_az, double max_az,
               double min_el, double max_el,
               double min_r, double max_r)
        : azimuth_resolution(az_res),
          elevation_resolution(el_res),
          min_azimuth(min_az),
          max_azimuth(max_az),
          min_elevation(min_el),
          max_elevation(max_el),
          min_range(min_r),
          max_range(max_r),
          timestamp(ros::Time::now()), // 初始化时间戳为当前时间
          local_position(Eigen::Vector3d::Zero()), // 初始化本地坐标为原点
          force_vector(Eigen::Vector3d::Zero())    // 初始化合力向量为零向量
    {
        // 计算网格数量
        num_azimuth_bins = static_cast<size_t>((max_azimuth - min_azimuth) / azimuth_resolution) + 1;
        num_elevation_bins = static_cast<size_t>((max_elevation - min_elevation) / elevation_resolution) + 1;
        
        // 初始化距离图和势场图
        dis_map.resize(num_azimuth_bins, std::vector<double>(num_elevation_bins, 0.0));
        pot_map.resize(num_azimuth_bins, std::vector<double>(num_elevation_bins, 0.0));
    }
};

/**
 * @class PointcloudProcessor
 * @brief 点云处理器类，负责处理Mid360激光雷达数据并生成极坐标场
 */
class PointcloudProcessor {
private:

    // ROS相关成员
    ros::NodeHandle nh_;

    // 订阅
    ros::Subscriber pointcloud_sub_;      // 点云订阅者

    // tf
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
    std::string lidar_topic_ ;            // 点云话题
    
    // 极坐标场数据
    PolarField polar_field_;              // 极坐标场
    mutable std::mutex polar_field_mutex_;// 极坐标场数据锁
    bool is_updated_;                     // 更新标志
    mutable std::mutex updated_mutex_;    // 更新标志锁

    //==========私有函数============
    void loadParams(); // 加载参数
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);//点云回调
    void filterPointcloud(const pcl::PointCloud<pcl::PointXYZ>& input, 
                         pcl::PointCloud<pcl::PointXYZ>& output);//过滤点云
    bool transformToBodyFrame(const pcl::PointCloud<pcl::PointXYZ>& input, 
                             pcl::PointCloud<pcl::PointXYZ>& output);//坐标系转换
    void generatePolarField(const pcl::PointCloud<pcl::PointXYZ>& cloud);//生成极坐标场
    void updatePolarFieldFromPoint(double x, double y, double z);//逐点更新极坐标场
    int angleToBinIndex(double angle, double min_angle, double max_angle, size_t num_bins);//角度映射

public:

    // 智能指针类型定义
    typedef boost::shared_ptr<PointcloudProcessor> Ptr;
    typedef boost::shared_ptr<const PointcloudProcessor> ConstPtr;

    //===========公共函数=================
    explicit PointcloudProcessor(ros::NodeHandle& nh);//构造函数
    ~PointcloudProcessor() = default;//析构函数
    const PolarField& getPolarField() const;//只读极坐标场
    bool isUpdated() const;//更新检测
    void resetUpdatedFlag();//重置更新标志
    int getAngleBinIndex(double angle, double min_angle, double max_angle, size_t num_bins);//获取角度索引

};

} // namespace avoid_planner 

#endif // AVOID_PLANNER_POINTCLOUD_PROCESSOR_H