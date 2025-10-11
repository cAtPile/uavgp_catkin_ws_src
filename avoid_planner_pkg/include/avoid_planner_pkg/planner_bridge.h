/**
 * @file planner_bridge.h
 * @brief 规划桥
 * @details 构建一个服务
 *          根据action,启动pointcloud_processor和potential_filed
 * @par apoc_pkg
 * @author apoc
 * @date 2025/10/7
 */

#ifndef AVOID_PLANNER_PLANNER_BRIDGE_H
#define AVOID_PLANNER_PLANNER_BRIDGE_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <avoid_planner_pkg/AvoidPlannerAction.h>  // 自定义Action消息
#include <Eigen/Dense>
#include <memory>
#include <mutex>

// 前向声明
namespace avoid_planner{
    class PointcloudProcessor;
    class PotentialFieldCalculator;
}

namespace avoid_planner{

// 极坐标场结构体定义
struct PolarField{
    // 角度分辨率配置
    double azimuth_resolution;  // 方位角分辨率(弧度)
    double elevation_resolution; // 仰角分辨率(弧度)
    
    // 角度范围
    double min_azimuth;         // 最小方位角(弧度)
    double max_azimuth;         // 最大方位角(弧度)
    double min_elevation;       // 最小仰角(弧度)
    double max_elevation;       // 最大仰角(弧度)

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
};

/**
 * @class AvoidPlannerBridge
 * @brief apoc和avoid_planner桥
 * @details 创建一个ROS action: 接收apoc的cmd控制avoid_planner启动停止
 *                         接收apoc的goal目标位置
 *          发布运动方向direction到apoc
 * @see class PointcloudProcessor 处理点云
 * @see class PotentialFieldCalculator 计算势场
 */
class PlannerBridge{
private:
    // ROS节点句柄
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // Action服务器
    actionlib::SimpleActionServer<avoid_planner_pkg::AvoidPlannerAction> as_;
    std::string action_name_;
    
    // Action消息
    avoid_planner_pkg::AvoidPlannerGoal current_goal_;
    avoid_planner_pkg::AvoidPlannerFeedback feedback_;
    avoid_planner_pkg::AvoidPlannerResult result_;

    // 子模块指针
    std::unique_ptr<PointcloudProcessor> pointcloud_processor_;
    std::unique_ptr<PotentialFieldCalculator> potential_field_calculator_;

    // 数据缓存
    PolarField current_polar_field_;
    Eigen::Vector3d current_direction_;
    std::mutex data_mutex_;  // 数据保护锁

    // 状态变量
    bool is_running_;
    ros::Rate update_rate_;

    // 回调函数
    void goalCallback();
    void preemptCallback();
    
    // 主处理循环
    void processLoop();
    
    // 数据处理函数
    bool updatePointcloud();
    bool calculatePotentialField();
    void publishDirection();

public:
    /**
     * @brief 构造函数
     * @param name Action服务器名称
     */
    PlannerBridge(const std::string& name);
    
    /**
     * @brief 析构函数
     */
    ~PlannerBridge();
    
    /**
     * @brief 启动规划桥
     */
    void start();
};

}  // namespace avoid_planner

#endif  // AVOID_PLANNER_PLANNER_BRIDGE_H