//星形搜索算法类声明
#ifndef MID360_AVOIDANCE_STAR_PLANNER_H
#define MID360_AVOIDANCE_STAR_PLANNER_H

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include "avoid_planner_pkg/cost_calculator.h"

namespace mid360_avoidance {

/**
 * @struct StarNode
 * @brief 星形搜索树的节点结构
 */
struct StarNode {
    Eigen::Vector3d position;          // 节点位置(ENU坐标系)
    Eigen::Vector3d direction;         // 节点方向向量
    double cost;                       // 从起点到该节点的累计成本
    double heuristic;                  // 启发式成本(到目标的估计成本)
    size_t parent_index;               // 父节点索引
    size_t depth;                      // 节点在搜索树中的深度
    
    // 构造函数
    StarNode() : cost(0.0), heuristic(0.0), parent_index(-1), depth(0) {}
    
    StarNode(const Eigen::Vector3d& pos, const Eigen::Vector3d& dir,
             double c, double h, size_t parent, size_t d)
        : position(pos), direction(dir), cost(c), heuristic(h),
          parent_index(parent), depth(d) {}
};

/**
 * @struct PathResult
 * @brief 路径规划结果结构
 */
struct PathResult {
    std::vector<Eigen::Vector3d> waypoints;  // 路径航点序列
    bool success;                            // 规划是否成功
    double total_cost;                       // 路径总成本
    ros::Time timestamp;                     // 时间戳
    
    // 构造函数
    PathResult() : success(false), total_cost(0.0) {}
};

/**
 * @class StarPlanner
 * @brief 星形搜索路径规划器类，基于成本矩阵生成局部最优避障路径
 */
class StarPlanner {
public:
    /**
     * @brief 构造函数
     */
    StarPlanner();
    
    /**
     * @brief 析构函数
     */
    ~StarPlanner() = default;
    
    /**
     * @brief 设置成本矩阵
     * @param cost_matrix 成本矩阵
     */
    void setCostMatrix(const CostMatrix& cost_matrix);
    
    /**
     * @brief 设置起点和目标点
     * @param start 起点位置(ENU坐标系)
     * @param goal 目标点位置(ENU坐标系)
     */
    void setStartAndGoal(const Eigen::Vector3d& start, const Eigen::Vector3d& goal);
    
    /**
     * @brief 设置无人机当前速度
     * @param velocity 速度向量(ENU坐标系)
     */
    void setCurrentVelocity(const Eigen::Vector3d& velocity);
    
    /**
     * @brief 执行星形搜索算法生成路径
     * @return 路径规划结果
     */
    PathResult searchPath();
    
    /**
     * @brief 设置搜索参数
     * @param max_depth 最大搜索深度
     * @param max_nodes 最大扩展节点数
     * @param step_size 每步搜索距离(m)
     * @param goal_tolerance 到达目标的容差(m)
     * @param heuristic_weight 启发式成本权重
     */
    void setParameters(size_t max_depth, size_t max_nodes, double step_size,
                      double goal_tolerance, double heuristic_weight);
    
    /**
     * @brief 获取最近一次的规划结果
     * @return 路径规划结果
     */
    const PathResult& getLastPath() const;

private:
    /**
     * @brief 计算启发式成本(到目标的估计成本)
     * @param node_position 节点位置
     * @return 启发式成本值
     */
    double calculateHeuristic(const Eigen::Vector3d& node_position);
    
    /**
     * @brief 将位置转换为对应的成本矩阵索引
     * @param direction 方向向量
     * @param[out] az_index 方位角索引
     * @param[out] el_index 仰角索引
     * @return 转换成功返回true
     */
    bool directionToCostIndices(const Eigen::Vector3d& direction,
                               size_t& az_index, size_t& el_index);
    
    /**
     * @brief 从方向向量计算方位角和仰角
     * @param direction 方向向量
     * @param[out] azimuth 方位角(弧度)
     * @param[out] elevation 仰角(弧度)
     */
    void directionToAngles(const Eigen::Vector3d& direction,
                          double& azimuth, double& elevation);
    
    /**
     * @brief 生成子节点的可能方向
     * @param parent_direction 父节点方向
     * @return 子节点可能的方向向量列表
     */
    std::vector<Eigen::Vector3d> generateChildDirections(const Eigen::Vector3d& parent_direction);
    
    /**
     * @brief 从搜索树回溯生成路径
     * @param goal_node_index 目标节点索引
     * @return 路径航点序列
     */
    std::vector<Eigen::Vector3d> backtrackPath(size_t goal_node_index);
    
    /**
     * @brief 检查节点是否接近目标
     * @param node_position 节点位置
     * @return 若接近目标返回true
     */
    bool isreachGoal(const Eigen::Vector3d& node_position);
    
    /**
     * @brief 平滑路径，减少抖动
     * @param path 原始路径
     * @return 平滑后的路径
     */
    std::vector<Eigen::Vector3d> smoothPath(const std::vector<Eigen::Vector3d>& path);

    // 输入数据
    CostMatrix cost_matrix_;              // 成本矩阵
    Eigen::Vector3d start_position_;      // 起点位置
    Eigen::Vector3d goal_position_;       // 目标位置
    Eigen::Vector3d current_velocity_;    // 当前速度向量
    bool has_valid_cost_matrix_;          // 成本矩阵是否有效
    
    // 搜索参数
    size_t max_depth_;                    // 最大搜索深度
    size_t max_nodes_;                    // 最大扩展节点数
    double step_size_;                    // 每步搜索距离(m)
    double goal_tolerance_;               // 到达目标的容差(m)
    double heuristic_weight_;             // 启发式成本权重
    
    // 搜索树
    std::vector<StarNode> search_tree_;   // 搜索树节点列表
    
    // 规划结果
    PathResult last_path_;                // 最近一次规划结果
    mutable std::mutex result_mutex_;     // 结果访问锁
    
    // 角度范围(与成本矩阵对应)
    double min_azimuth_;                  // 最小方位角(弧度)
    double max_azimuth_;                  // 最大方位角(弧度)
    double min_elevation_;                // 最小仰角(弧度)
    double max_elevation_;                // 最大仰角(弧度)
};

} // namespace mid360_avoidance

#endif // MID360_AVOIDANCE_STAR_PLANNER_H
