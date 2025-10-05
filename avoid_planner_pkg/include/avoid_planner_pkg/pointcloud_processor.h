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

namespace avoid_planner {

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

    double max_range;  // 这是新增的成员，用于存储最大传感器距离
    
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
        max_range(50.0),  // 新增：初始化最大传感器距离（如Mid360的最大探测距离为50米）
        min_azimuth(-M_PI),
        max_azimuth(M_PI),
        min_elevation(-0.122),      // -°(弧度)
        max_elevation(0.984),       // +°(弧度)
        num_azimuth_bins(360),
        num_elevation_bins(12),
        data(360, std::vector<double>(12, INFINITY)) {}
};

/**
 * @class PointcloudProcessor
 * @brief 点云处理器类，负责处理Mid360激光雷达数据并生成障碍物直方图
 */
class PointcloudProcessor {

private:

    // ROS相关成员
    ros::NodeHandle nh_;

    //订阅
    ros::Subscriber pointcloud_sub_;      // 点云订阅者

    //tf
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
    std::string lidar_topic_ ;             // 点云话题
    
    // 直方图数据
    PolarHistogram histogram_;            // 极坐标直方图
    mutable std::mutex histogram_mutex_;  // 直方图数据锁
    bool is_updated_;                     // 更新标志
    mutable std::mutex updated_mutex_;    // 更新标志锁

    //==========私有函数============
    void loadParams(); //加载参数
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);//点云回调
    void filterPointcloud(const pcl::PointCloud<pcl::PointXYZ>& input, 
                         pcl::PointCloud<pcl::PointXYZ>& output);//过滤点云
    bool transformToBodyFrame(const pcl::PointCloud<pcl::PointXYZ>& input, 
                             pcl::PointCloud<pcl::PointXYZ>& output);//坐标系转换
    void generatePolarHistogram(const pcl::PointCloud<pcl::PointXYZ>& cloud);//生成极坐标直方图
    void updateHistogramFromPoint(double x, double y, double z);//逐点更新直方图
    int angleToBinIndex(double angle, double min_angle, double max_angle, size_t num_bins);//角度映射

public:

    // 智能指针类型定义
    typedef boost::shared_ptr<PointcloudProcessor> Ptr;
    typedef boost::shared_ptr<const PointcloudProcessor> ConstPtr;

    //===========公共函数=================
    explicit PointcloudProcessor(ros::NodeHandle& nh);//构造函数
    ~PointcloudProcessor() = default;//析构函数
    const PolarHistogram& getHistogram() const;//只读直方图
    bool isUpdated() const;//更新检测
    void resetUpdatedFlag();//重置更新标致
    int getAngleBinIndex(double angle, double min_angle, double max_angle, size_t num_bins);//获取角度索引

};

} // namespace avoid_planner 

#endif // AVOID_PLANNER_POINTCLOUD_PROCESSOR_H
