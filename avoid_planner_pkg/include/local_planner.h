#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <ros/time.h>

namespace avoidance {

/**
 * @brief 路径规划器输出结果结构体
 */
struct PlannerOutput {
    Eigen::Vector3f next_setpoint;  // 下一步目标坐标
    bool is_goal_reached;           // 是否到达目标点
    float distance_to_goal;         // 到目标点的距离
    ros::Time planning_time;        // 规划时间戳
};

/**
 * @brief 局部路径规划器类，核心功能是根据点云和目标点计算下一步坐标
 */
class LocalPlanner {
public:
    /**
     * @brief 构造函数
     */
    LocalPlanner();
    
    /**
     * @brief 析构函数
     */
    ~LocalPlanner() = default;

    /**
     * @brief 设置当前位置
     * @param pos 当前位置坐标(x,y,z)
     */
    void setCurrentPosition(const Eigen::Vector3f& pos);

    /**
     * @brief 设置目标点
     * @param goal 目标点坐标(x,y,z)
     */
    void setGoal(const Eigen::Vector3f& goal);

    /**
     * @brief 设置障碍物点云
     * @param cloud 经过预处理的点云数据
     */
    void setObstacleCloud(const pcl::PointCloud<pcl::PointXYZI>& cloud);

    /**
     * @brief 设置规划参数
     * @param safe_distance 安全距离(米)
     * @param step_size 步长(米)
     * @param goal_tolerance 目标容忍度(米)
     */
    void setParams(float safe_distance = 0.5f, 
                  float step_size = 0.3f, 
                  float goal_tolerance = 0.2f);

    /**
     * @brief 执行路径规划，计算下一步坐标
     * @return 规划结果，包含下一步坐标和状态信息
     */
    PlannerOutput computeNextSetpoint();

private:
    /**
     * @brief 检查直线路径是否存在障碍物
     * @param start 起点
     * @param end 终点
     * @return 有障碍物返回true，否则返回false
     */
    bool isPathBlocked(const Eigen::Vector3f& start, const Eigen::Vector3f& end);

    /**
     * @brief 生成避障备选点
     * @return 备选点集合
     */
    std::vector<Eigen::Vector3f> generateAvoidanceCandidates();

    /**
     * @brief 评估备选点优劣
     * @param candidates 备选点集合
     * @return 最优备选点索引
     */
    int evaluateCandidates(const std::vector<Eigen::Vector3f>& candidates);

    /**
     * @brief 计算两点间距离
     * @param a 点A
     * @param b 点B
     * @return 距离值
     */
    float distance(const Eigen::Vector3f& a, const Eigen::Vector3f& b);

    // 状态变量
    Eigen::Vector3f current_pos_;    // 当前位置
    Eigen::Vector3f goal_pos_;       // 目标位置
    pcl::PointCloud<pcl::PointXYZI> obstacle_cloud_;  // 障碍物点云
    bool has_goal_;                  // 是否设置了目标点
    bool has_cloud_;                 // 是否收到点云数据

    // 规划参数
    float safe_distance_;            // 安全距离
    float step_size_;                // 步长
    float goal_tolerance_;           // 目标容忍度
    float max_avoidance_distance_;   // 最大避障偏移距离
};

}  // namespace avoidance

#endif  // LOCAL_PLANNER_H
    