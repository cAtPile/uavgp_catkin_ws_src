/**
 * @file utils.h
 * @brief 通用工具
 */
#ifndef AVOID_PLANNER_UTILS_H
#define AVOID_PLANNER_UTILS_H

#include <Eigen/Dense>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <vector>
#include <string>


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

#endif // AVOID_PLANNER_UTILS_H
