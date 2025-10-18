/**
 * @file avoid_planner.h
 * @brief 避障规划器头文件
 * @details 定义避障规划器的核心类和数据结构，包含势场计算、点云处理和ROS接口相关声明
 * @par avoid_planner_pkg
 * @author apoc
 * @date 2025/10/18
 */

#ifndef AVOID_PLANNER_H
#define AVOID_PLANNER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <avoid_planner_pkg/AvoidPlannerAction.h>  // 自定义Action消息
#include <Eigen/Dense>
#include <mutex>
#include <geometry_msgs/Vector3Stamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <thread>
#include <cmath>

/**
 * @struct PolarField
 * @brief 极坐标场结构体
 * @details 存储极坐标直方图数据、角度参数和势场计算结果
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
    double min_range;   // 最小传感器距离(m)
    
    // 网格尺寸
    size_t num_azimuth_bins;    // 方位角网格数量
    size_t num_elevation_bins;  // 仰角网格数量
    
    // 直方图数据: [azimuth][elevation] = 障碍物距离(m)
    std::vector<std::vector<double>> obstacle_distances;  // 原dis_map重命名，更清晰

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
        obstacle_distances.resize(num_azimuth_bins, std::vector<double>(num_elevation_bins, INFINITY));
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
          timestamp(ros::Time::now()),
          local_position(Eigen::Vector3d::Zero()),
          force_vector(Eigen::Vector3d::Zero())
    {
        // 计算网格数量
        num_azimuth_bins = static_cast<size_t>((max_azimuth - min_azimuth) / azimuth_resolution) + 1;
        num_elevation_bins = static_cast<size_t>((max_elevation - min_elevation) / elevation_resolution) + 1;
        
        // 初始化距离图和势场图
        obstacle_distances.resize(num_azimuth_bins, std::vector<double>(num_elevation_bins, INFINITY));
        pot_map.resize(num_azimuth_bins, std::vector<double>(num_elevation_bins, 0.0));
    }
};

/**
 * @class AvoidPlanner
 * @brief 避障规划器类
 * @details 实现基于势场法的避障规划，接收点云数据和目标位置，计算运动方向
 */
class AvoidPlanner{
private:

    //===========ROS节点====================
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    //===========Action服务器================
    actionlib::SimpleActionServer<avoid_planner_pkg::AvoidPlannerAction> as_;
    std::string action_name_;
    
    //============Action消息===================
    avoid_planner_pkg::AvoidPlannerGoal current_goal_;
    avoid_planner_pkg::AvoidPlannerFeedback feedback_;
    avoid_planner_pkg::AvoidPlannerResult result_;

    //=============topic订阅===================
    ros::Subscriber pointcloud_sub_;      // 点云订阅者
    std::string lidar_topic_;             // 激光雷达话题名

    //=============数据缓存=====================
    PolarField current_polar_field_;
    Eigen::Vector3d current_direction_;
    std::vector<std::vector<double>> pot_map_;  // 势场图缓存
    std::mutex data_mutex_;       // 数据保护锁
    std::mutex histogram_mutex_;  // 直方图数据锁
    std::mutex updated_mutex_;    // 更新标志锁
    bool is_updated_;             // 数据更新标志

    // 目标角度参数（用于默认方向计算）
    double goal_az_;              // 目标方位角
    double goal_el_;              // 目标俯仰角

    //==============参数=====================
    /*点云处理参数*/
    double max_sensor_range_;             // 最大感知距离(m)
    double min_sensor_range_;             // 最小感知距离(m)
    double voxel_grid_size_;              // 体素滤波分辨率(m)
    int statistical_filter_mean_k_;       // 统计滤波邻域点数
    double statistical_filter_std_dev_;   // 统计滤波标准差阈值

    // 状态变量
    bool is_running_;
    ros::Rate update_rate_;

    // 内部函数
    void loadParams();                     // 加载参数
    void updatePFpoint(double x, double y, double z);  // 更新极坐标场点

    // 回调函数
    void goalCB();
    void pointcloudCB(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // 主处理循环
    void processLoop();
    
    // 数据处理函数
    void publishDirection();

    //=============点云处理函数============
    void filterPointcloud(const pcl::PointCloud<pcl::PointXYZ>& input, 
                          pcl::PointCloud<pcl::PointXYZ>& output);
    bool tfBodyframe(const pcl::PointCloud<pcl::PointXYZ>& input, 
                     pcl::PointCloud<pcl::PointXYZ>& output);
    void generatePFdismap(const pcl::PointCloud<pcl::PointXYZ>& cloud);

    //=============势场计算函数============
    double calculateAtt(double goal_dis);  // 计算引力
    double calculateRep(size_t az_idx, size_t el_idx);  // 计算斥力
    void calculateTotal(size_t az_idx, size_t el_idx, double att_force, double rep_force);  // 计算合力
    void generatePFpotmap(double goal_az, double goal_el, double goal_dis);  // 生成势场图
    void generateForceDir();  // 生成合力方向
    int anglebinIndex(double angle, double min_angle, double max_angle, size_t num_bins);  // 计算角度网格索引

public:
    AvoidPlanner();  // 构造函数
    ~AvoidPlanner(); // 析构函数
    void start();    // 启动规划器
    Eigen::Vector3d getCurrentDir();  // 获取当前运动方向
    PolarField getCurrentPolarField();  // 获取当前极坐标场
};

#endif  // AVOID_PLANNER_H