//通用工具函数声明
#ifndef MID360_AVOIDANCE_UTILS_H
#define MID360_AVOIDANCE_UTILS_H

#include <Eigen/Dense>
#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <vector>
#include <string>

namespace mid360_avoidance {

/**
 * @struct Params
 * @brief 算法参数结构体，包含所有模块的配置参数
 */
struct Params {
    // 点云处理参数
    double pointcloud_min_range;          // 点云最小距离(m)
    double pointcloud_max_range;          // 点云最大距离(m)
    double voxel_grid_size;               // 体素滤波尺寸(m)
    int statistical_filter_mean_k;        // 统计滤波邻域点数
    double statistical_filter_std_dev;    // 统计滤波标准差阈值
    std::string lidar_frame_id;           // 激光雷达坐标系ID
    std::string body_frame_id;            // 机体坐标系ID
    
    // 成本计算参数
    double safety_distance;               // 安全距离(m)
    double obstacle_weight;               // 障碍物成本权重
    double goal_weight;                   // 目标方向成本权重
    double smoothness_weight;             // 平滑性成本权重
    
    // 星形搜索参数
    size_t star_max_depth;                // 最大搜索深度
    size_t star_max_nodes;                // 最大扩展节点数
    double star_step_size;                // 搜索步长(m)
    double star_goal_tolerance;           // 目标容差(m)
    double star_heuristic_weight;         // 启发式权重
    
    // 局部规划器参数
    double planner_update_rate;           // 规划更新频率(Hz)
    double local_goal_distance;           // 局部目标距离(m)
    double path_safety_threshold;         // 路径安全阈值(m)
    int max_failure_count;                // 最大失败次数
    
    // 构造函数(默认参数)
    Params() : pointcloud_min_range(0.5), pointcloud_max_range(50.0),
              voxel_grid_size(0.1), statistical_filter_mean_k(10),
              statistical_filter_std_dev(0.1), lidar_frame_id("mid360_link"),
              body_frame_id("base_link"), safety_distance(1.0),
              obstacle_weight(0.5), goal_weight(0.3), smoothness_weight(0.2),
              star_max_depth(5), star_max_nodes(50), star_step_size(0.5),
              star_goal_tolerance(0.3), star_heuristic_weight(1.0),
              planner_update_rate(10.0), local_goal_distance(2.0),
              path_safety_threshold(0.5), max_failure_count(3) {}
};

/**
 * @brief 从四元数提取偏航角(弧度)
 * @param quat 四元数消息
 * @return 偏航角(弧度，范围[-π, π])
 */
Eigen::Vector3d quaternionToYaw(const geometry_msgs::Quaternion& quat);

/**
 * @brief 计算两点之间的欧氏距离
 * @param a 点A坐标
 * @param b 点B坐标
 * @return 两点间距离(m)
 */
double calculateDistance(const Eigen::Vector3d& a, const Eigen::Vector3d& b);

/**
 * @brief 对路径进行平滑处理，减少航点抖动
 * @param path 输入路径，处理后结果直接修改该路径
 * @param window_size 平滑窗口大小，默认为3
 */
void smoothPath(std::vector<Eigen::Vector3d>& path, size_t window_size = 3);

/**
 * @brief 从ROS参数服务器加载算法参数
 * @param nh ROS节点句柄
 * @param params 输出参数结构体
 * @return 加载成功返回true，否则返回false
 */
bool loadParameters(ros::NodeHandle& nh, Params& params);

/**
 * @brief 将角度归一化到[-π, π]范围
 * @param angle 输入角度(弧度)
 * @return 归一化后的角度
 */
double normalizeAngle(double angle);

/**
 * @brief 将ENU坐标系下的位置转换为ROS消息
 * @param position ENU坐标
 * @param frame_id 坐标系ID
 * @return 转换后的PoseStamped消息
 */
geometry_msgs::PoseStamped vectorToPoseStamped(const Eigen::Vector3d& position,
                                              const std::string& frame_id);

} // namespace mid360_avoidance

#endif // MID360_AVOIDANCE_UTILS_H
