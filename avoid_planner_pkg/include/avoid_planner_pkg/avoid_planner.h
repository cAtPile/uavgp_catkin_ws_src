/**
 * @file avoid_planner.h
 * @brief 
 * @details 
 * @par apoc_pkg
 * @author apoc
 * @date 2025/10/7
 */

#ifndef AVOID_PLANNER_H
#define AVOID_PLANNER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <avoid_planner_pkg/AvoidPlannerAction.h>  // 自定义Action消息
#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <geometry_msgs/Vector3Stamped.h>
#include <thread>

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
 * @class AvoidPlanner
 * @brief 避障规划器
 * @details 创建一个ROS Action: 接收apoc的cmd控制avoid_planner启动停止
 *                         接收apoc的goal目标位置
 *          订阅点云话题
 *          接受/处理点云数据
 *          计算势场和力
 *          发布运动方向direction到apoc
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
    ros::Subscriber local_pos_sub_;

    //=============数据缓存=====================
    PolarField current_polar_field_;
    Eigen::Vector3d current_direction_;
    std::mutex data_mutex_;  // 数据保护锁

    //==============参数=====================
    /*点云处理参数*/
    double max_sensor_range_;             // 最大感知距离(m)
    double min_sensor_range_;             // 最小感知距离(m)
    double voxel_grid_size_;              // 体素滤波分辨率(m)

    // 状态变量
    bool is_running_;
    ros::Rate update_rate_;

    // 回调函数
    void goalCB();
    void pointcloudCB();
    //void preemptCallback();

    // 主处理循环
    void processLoop();
    
    // 数据处理函数
    bool updatePointcloud();
    bool calculatePotentialField();
    void publishDirection();

    //=============点云处理函数============
    void filterPC();//过滤点云
    bool tfBodyFrame();//坐标转换
    void generatePFdismap();//生成dismap

public:

    PlannerBridge(const std::string& name);//构造函数
    ~PlannerBridge();//析构函数
    void start();//启动器
    Eigen::Vector3d getCurrentDir();//获取当前方向
    PolarField getCurrentPolarField();//获取当前极坐标地图
};

#endif  // AVOID_PLANNER_H