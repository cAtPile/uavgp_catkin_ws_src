#ifndef AVOID_PLANNER_POTENTIAL_FIELD_H
#define AVOID_PLANNER_POTENTIAL_FIELD_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <cmath>

namespace avoid_planner {
    
/**
 * @struct PolarHistogram
 * @brief 极坐标直方图结构，存储每个角度网格中的障碍物距离
 */
struct PolarHistogram {
    // 角度分辨率配置
    double azimuth_resolution;  // 方位角分辨率(弧度)
    double elevation_resolution; // 仰角分辨率(弧度)
    
    // 角度范围
    double min_azimuth;         // 最小方位角(弧度)
    double max_azimuth;         // 最大方位角(弧度)
    double min_elevation;       // 最小仰角(弧度)
    double max_elevation;       // 最大仰角(弧度)

    double max_range;  // 最大传感器距离
    
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
        max_range(50.0),  // 初始化最大传感器距离
        min_azimuth(-M_PI),
        max_azimuth(M_PI),
        min_elevation(-0.122),      // 约-7°(弧度)
        max_elevation(0.984),       // 约+56°(弧度)
        num_azimuth_bins(360),
        num_elevation_bins(12),
        data(360, std::vector<double>(12, INFINITY)) {}
};

/**
 * @struct PotentialGrid
 * @brief 势场网格结构，存储每个角度网格中的势场信息
 */
struct PotentialGrid {
    // 角度分辨率配置
    double azimuth_resolution;  // 方位角分辨率(弧度)
    double elevation_resolution; // 仰角分辨率(弧度)
    
    // 角度范围
    double min_azimuth;         // 最小方位角(弧度)
    double max_azimuth;         // 最大方位角(弧度)
    double min_elevation;       // 最小仰角(弧度)
    double max_elevation;       // 最大仰角(弧度)
    
    // 网格尺寸
    size_t num_azimuth_bins;    // 方位角网格数量
    size_t num_elevation_bins;  // 仰角网格数量

    // 势场数据: [azimuth][elevation] = 势场力大小
    std::vector<std::vector<double>> data;

    Eigen::Vector3d local_position; // 本地坐标
    Eigen::Vector3d force_vector;   // 合力向量
    
    ros::Time timestamp; // 时间戳

    // 构造函数
    PotentialGrid() :
        azimuth_resolution(0.01745),
        elevation_resolution(0.0873),
        min_azimuth(-M_PI),
        max_azimuth(M_PI),
        min_elevation(-0.122),
        max_elevation(0.984),
        num_azimuth_bins(360),
        num_elevation_bins(12),
        data(360, std::vector<double>(12, 0.0)),
        local_position(Eigen::Vector3d::Zero()),
        force_vector(Eigen::Vector3d::Zero()) {}
};

/**
 * @class PotentialFieldCalculator
 * @brief 人工势场法计算类，用于三维空间中的避障规划
 */
class PotentialFieldCalculator {
private:
    // 数据成员
    double att_gain_;           // 引力增益系数
    double rep_gain_;           // 斥力增益系数
    double rep_radius_;         // 斥力影响半径
    double max_force_;          // 最大合力限制
    double field_resolution_;   // 势场分辨率(m)
    double field_range_;        // 势场作用范围(m)
    double attract_range_;      // 引力影响范围
    Eigen::Vector3d current_pose_;  // 当前位置(x, y, z)
    Eigen::Vector3d goal_pose_;     // 目标位置(x, y, z)
    Eigen::Vector3d force_direction_;  // 合力方向
    Eigen::Vector3d polar_goal_;    // 目标的极坐标位置（az，el，dis）
    PotentialGrid current_field_;   // 当前势场
    PolarHistogram current_histogram_; // 当前极坐标直方图
    bool is_updated_;           // 势场是否已更新的标志

    //===========成员函数=====================
    /**
     * @brief 从参数服务器加载参数
     */
    void loadParams();

    /**
     * @brief 获取极坐标直方图（从点云处理器或传感器）
     * @return 极坐标直方图
     */
    PolarHistogram getPolarHistogram();

    /**
     * @brief 计算引力
     * @param az 方位角
     * @param el 仰角
     * @return 引力大小
     */
    double calculateAttractiveForce(double az, double el);

    /**
     * @brief 计算斥力
     * @param az 方位角
     * @param el 仰角
     * @return 斥力大小
     */
    double calculateRepulsiveForce(double az, double el);

    /**
     * @brief 计算合力
     * @param az 方位角
     * @param el 仰角
     * @return 合力大小
     */
    double calculateTotalForce(double az, double el);

    /**
     * @brief 生成合力方向
     * @return 合力方向向量
     */
    Eigen::Vector3d generateTotalForce();

    /**
     * @brief 获取目标点位置并转换为极坐标
     */
    void getGoal();

    /**
     * @brief 生成引力扇区（限制引力作用角度范围）
     */
    void generateAttractiveFan();

    /**
     * @brief 生成势场图
     * @return 生成的势场网格
     */
    PotentialGrid generatePotentialField();

public:
    PotentialFieldCalculator(ros::NodeHandle& nh);//构造函数
    ~PotentialFieldCalculator()= default;//析构

    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */


    /**
     * @brief 析构函数
     */




    /**
     * @brief 逐步更新势场图（用于动态环境）
     * @return 更新比例
     */
    double stepPotentialField();

    /**
     * @brief 获取当前势场
     * @return 当前势场网格
     */
    PotentialGrid getPotentialField();

    /**
     * @brief 获取合力方向
     * @return 合力方向向量
     */
    Eigen::Vector3d getForceDirection();

    /**
     * @brief 更新当前位置
     * @param pose 新的位置
     */
    void updateCurrentPose(const Eigen::Vector3d& pose);

    /**
     * @brief 设置目标位置
     * @param goal 目标位置
     */
    void setGoalPose(const Eigen::Vector3d& goal);
};

}  // namespace avoid_planner

#endif  // AVOID_PLANNER_POTENTIAL_FIELD_H