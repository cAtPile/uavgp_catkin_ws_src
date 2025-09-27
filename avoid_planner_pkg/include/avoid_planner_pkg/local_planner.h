#ifndef MID360_AVOIDANCE_LOCAL_PLANNER_H
#define MID360_AVOIDANCE_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include "avoid_planner_pkg/pointcloud_processor.h"
#include "avoid_planner_pkg/cost_calculator.h"
#include "avoid_planner_pkg/star_planner.h"
#include "avoid_planner_pkg/mavros_interface.h"
#include <ros/timer.h>  // 让编译器认识 ros::TimerEvent 类型

namespace mid360_avoidance {

/**
 * @struct PlannerConfig
 * @brief 规划器配置参数结构
 */
struct PlannerConfig {
    // 规划周期(Hz)
    double update_rate;
    // 局部目标点距离(m)
    double local_goal_distance;
    // 路径安全检查距离阈值(m)
    double path_safety_threshold;
    // 最大规划失败次数
    int max_failure_count;
    
    // 构造函数(默认参数)
    PlannerConfig() : update_rate(10.0), local_goal_distance(2.0),
                     path_safety_threshold(0.5), max_failure_count(3) {}
};

/**
 * @class LocalPlanner
 * @brief 局部路径规划器主类，协调各模块工作并输出最终避障路径
 */
class LocalPlanner {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    explicit LocalPlanner(ros::NodeHandle& nh);
    
    /**
     * @brief 析构函数
     */
    ~LocalPlanner() = default;
    
    /**
     * @brief 启动规划器主循环
     */
    void start();
    
    /**
     * @brief 设置全局路径
     * @param global_path 全局路径消息
     */
    void setGlobalPath(const nav_msgs::Path::ConstPtr& global_path);
    
    /**
     * @brief 获取当前局部路径
     * @return 局部路径消息
     */
    nav_msgs::Path getLocalPath() const;
    
    /**
     * @brief 检查规划器是否正常运行
     * @return 正常运行返回true
     */
    bool isRunning() const;
    
    /**
     * @brief 停止规划器
     */
    void stop();

private:
    /**
     * @brief 规划器主循环函数
     */
    void plannerLoop(const ros::TimerEvent& event);
    
    /**
     * @brief 从全局路径中获取局部目标点
     * @return 局部目标点坐标(ENU坐标系)
     */
    Eigen::Vector3d getLocalGoal();
    
    /**
     * @brief 检查生成的路径是否安全
     * @param path 待检查的路径
     * @return 安全返回true，否则返回false
     */
    bool isPathSafe(const std::vector<Eigen::Vector3d>& path);
    
    /**
     * @brief 发布局部路径可视化消息
     */
    void publishLocalPath();
    
    /**
     * @brief 处理规划失败情况
     */
    void handlePlanningFailure();
    
    /**
     * @brief 加载配置参数
     */
    void loadParameters();

    // ROS相关成员
    ros::NodeHandle nh_;                      // ROS节点句柄
    ros::Subscriber global_path_sub_;         // 全局路径订阅者
    ros::Publisher local_path_pub_;           // 局部路径发布者
    ros::Timer planner_timer_;                // 规划器定时器
    bool is_running_;                         // 规划器运行状态
    
    // 子模块
    std::unique_ptr<PointcloudProcessor> pointcloud_processor_;  // 点云处理器
    std::unique_ptr<CostCalculator> cost_calculator_;            // 成本计算器
    std::unique_ptr<StarPlanner> star_planner_;                  // 星形搜索规划器
    std::unique_ptr<MavrosInterface> mavros_interface_;          // MAVROS接口
    
    // 路径数据
    nav_msgs::Path global_path_;              // 全局路径
    nav_msgs::Path local_path_;               // 局部路径
    size_t global_path_current_index_;        // 当前全局路径索引
    mutable std::mutex path_mutex_;           // 路径数据锁
    
    // 规划状态
    int planning_failure_count_;              // 规划失败计数器
    bool is_goal_reached_;                    // 目标是否到达
    
    // 配置参数
    PlannerConfig config_;                    // 规划器配置
};

} // namespace mid360_avoidance

#endif // MID360_AVOIDANCE_LOCAL_PLANNER_H
