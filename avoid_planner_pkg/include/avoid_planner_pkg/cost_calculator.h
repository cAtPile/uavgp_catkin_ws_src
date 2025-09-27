//成本计算模块声明
#ifndef MID360_AVOIDANCE_COST_CALCULATOR_H
#define MID360_AVOIDANCE_COST_CALCULATOR_H

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <mutex>
#include "mid360_avoidance/pointcloud_processor.h"

namespace mid360_avoidance {

/**
 * @struct CostMatrix
 * @brief 成本矩阵结构，存储每个角度网格的避障成本
 */
struct CostMatrix {
    // 与直方图对应的角度分辨率和网格数量
    size_t num_azimuth_bins;    // 方位角网格数量
    size_t num_elevation_bins;  // 仰角网格数量
    
    // 成本数据: [azimuth][elevation] = 成本值(0~1.0)
    std::vector<std::vector<double>> data;
    
    // 时间戳
    ros::Time timestamp;
    
    // 构造函数
    CostMatrix() : num_azimuth_bins(360), num_elevation_bins(7),
                  data(360, std::vector<double>(7, 0.0)) {}
};

/**
 * @class CostCalculator
 * @brief 成本计算器类，基于障碍物直方图和目标位置计算各方向的避障成本
 */
class CostCalculator {
public:
    /**
     * @brief 构造函数
     */
    CostCalculator();
    
    /**
     * @brief 析构函数
     */
    ~CostCalculator() = default;
    
    /**
     * @brief 设置障碍物直方图数据
     * @param histogram 极坐标直方图
     */
    void setHistogram(const PolarHistogram& histogram);
    
    /**
     * @brief 设置局部目标点(ENU坐标系)
     * @param goal 目标点坐标
     */
    void setGoal(const Eigen::Vector3d& goal);
    
    /**
     * @brief 设置无人机当前位置和速度(ENU坐标系)
     * @param position 当前位置
     * @param velocity 当前速度
     */
    void setRobotState(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity);
    
    /**
     * @brief 计算成本矩阵
     * @return 计算成功返回true，否则返回false
     */
    bool computeCostMatrix();
    
    /**
     * @brief 获取计算后的成本矩阵
     * @return 成本矩阵引用
     */
    const CostMatrix& getCostMatrix() const;
    
    /**
     * @brief 检查是否有新的成本矩阵数据
     * @return 若有新数据则返回true，否则返回false
     */
    bool isUpdated() const;
    
    /**
     * @brief 重置更新标志
     */
    void resetUpdatedFlag();
    
    /**
     * @brief 设置成本计算参数
     * @param safety_distance 安全距离(m)
     * @param obstacle_weight 障碍物成本权重(0~1)
     * @param goal_weight 目标方向成本权重(0~1)
     * @param smoothness_weight 平滑性成本权重(0~1)
     */
    void setParameters(double safety_distance, double obstacle_weight,
                      double goal_weight, double smoothness_weight);

private:
    /**
     * @brief 计算单个角度方向的成本
     * @param azimuth 方位角(弧度)
     * @param elevation 仰角(弧度)
     * @param bin_az 方位角网格索引
     * @param bin_el 仰角网格索引
     * @return 成本值(0~1.0)
     */
    double calculateDirectionCost(double azimuth, double elevation,
                                 size_t bin_az, size_t bin_el);
    
    /**
     * @brief 计算障碍物距离成本
     * @param distance 障碍物距离(m)
     * @return 成本值(0~1.0)
     */
    double obstacleDistanceCost(double distance);
    
    /**
     * @brief 计算目标方向偏差成本
     * @param azimuth 方位角(弧度)
     * @param elevation 仰角(弧度)
     * @return 成本值(0~1.0)
     */
    double goalDirectionCost(double azimuth, double elevation);
    
    /**
     * @brief 计算运动平滑性成本
     * @param azimuth 方位角(弧度)
     * @param elevation 仰角(弧度)
     * @return 成本值(0~1.0)
     */
    double smoothnessCost(double azimuth, double elevation);
    
    /**
     * @brief 平滑成本矩阵，减少局部极小值
     */
    void smoothCostMatrix();
    
    /**
     * @brief 将角度坐标转换为单位向量
     * @param azimuth 方位角(弧度)
     * @param elevation 仰角(弧度)
     * @return 单位方向向量
     */
    Eigen::Vector3d anglesToDirectionVector(double azimuth, double elevation);

    // 输入数据
    PolarHistogram histogram_;          // 障碍物直方图
    Eigen::Vector3d goal_;              // 目标点
    Eigen::Vector3d robot_position_;    // 机器人位置
    Eigen::Vector3d robot_velocity_;    // 机器人速度
    bool histogram_updated_;            // 直方图更新标志
    
    // 输出数据
    CostMatrix cost_matrix_;            // 成本矩阵
    bool is_updated_;                   // 成本矩阵更新标志
    
    // 计算参数
    double safety_distance_;            // 安全距离(m)
    double max_sensor_range_;           // 最大感知距离(m)
    double obstacle_weight_;            // 障碍物成本权重
    double goal_weight_;                // 目标方向成本权重
    double smoothness_weight_;          // 平滑性成本权重
    
    // 目标方向角(相对于机体坐标系)
    double goal_azimuth_;               // 目标方位角(弧度)
    double goal_elevation_;             // 目标仰角(弧度)
    
    // 线程安全控制
    mutable std::mutex data_mutex_;     // 数据访问锁
    mutable std::mutex update_mutex_;   // 更新标志锁
};

} // namespace mid360_avoidance

#endif // MID360_AVOIDANCE_COST_CALCULATOR_H
