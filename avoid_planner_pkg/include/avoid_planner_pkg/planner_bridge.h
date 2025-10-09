/**
 * @file planner_brideg.h
 * @brief 规划桥
 * @details 构建一个服务
 *          根据action,启动pointcloud_processor和potential_filed
 * @par apoc_pkg
 * @author apoc
 * @date 2025/10/7
 */

#ifndef AVOID_PLANNER_PLANNER_BRIDGE_H
#define AVOID_PLANNER_PLANNER_BRIDGE_H

# include<ros/ros.h>

namespace avoid_planner{

//构建结构体polar_field
/**
 * @struct PolarField
 * @brief 极坐标场
 * @details 存储地图信息和网格信息
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

    double max_range;   // 用于存储最大传感器距离(m)
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
 * @details 创建一个ROS action: 收apoc的cmd控制avoid_planner启动停止
 *                         收apoc的goal目标位置
 *          发运动方向direction到apoc
 * @see class PointcloudProcessor 处理点云
 * @see class PotentialFieldCalculator 计算势场
 */
class planner_bridge{
private:
    /* data */
public:
    planner_bridge(/* args */);
    ~planner_bridge();
};

}

#endif