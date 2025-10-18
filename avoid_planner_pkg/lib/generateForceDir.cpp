/**
 * @file generateForceDir.cpp
 * @brief
 * @details
 * @author apoc
 * @date 2025/10/18
 */  
#include "avoid_planner_pkg/avoid_planner.h"

/**
 * @brief 计算并生成机器人最终运动的合力方向
 * @details 遍历势场图所有网格，将每个网格的合力转换为笛卡尔坐标系下的矢量分量，累加后归一化得到最终方向
 * @note 需确保势场图（pot_map_）已在calculateTotal中完成合力计算
 */
void AvoidPlanner::generateForceDir() {
    // 初始化总体合力矢量（笛卡尔坐标系，x/y/z分别对应水平前向、水平侧向、垂直方向）
    Eigen::Vector3d total_force = Eigen::Vector3d::Zero();

    // 获取势场图网格尺寸（从极坐标场结构体中获取，确保与势场图维度一致）
    const size_t num_az_bins = current_polar_field_.num_azimuth_bins;
    const size_t num_el_bins = current_polar_field_.num_elevation_bins;
    const double az_res = current_polar_field_.azimuth_resolution;    // 方位角分辨率（弧度）
    const double el_res = current_polar_field_.elevation_resolution;  // 俯仰角分辨率（弧度）
    const double min_az = current_polar_field_.min_azimuth;          // 最小方位角（弧度，默认-π）
    const double min_el = current_polar_field_.min_elevation;        // 最小俯仰角（弧度，默认-7°对应值）

    // 遍历所有角度网格，累加各网格的合力矢量
    for (size_t az_idx = 0; az_idx < num_az_bins; ++az_idx) {
        for (size_t el_idx = 0; el_idx < num_el_bins; ++el_idx) {
            // 1. 获取当前网格的合力大小（从势场图中读取）
            double grid_force = pot_map_[az_idx][el_idx];
            if (grid_force < 1e-6) {  // 过滤极小值，避免无效计算
                continue;
            }

            // 2. 计算当前网格对应的实际角度（弧度）
            double current_az = min_az + az_idx * az_res;  // 方位角（绕z轴，-π~π）
            double current_el = min_el + el_idx * el_res;  // 俯仰角（绕y轴，-7°~56°对应值）

            // 3. 将极坐标下的合力转换为笛卡尔坐标系矢量分量
            Eigen::Vector3d grid_force_vec;
            // x分量：水平前向（方位角余弦 × 俯仰角余弦 × 合力大小）
            grid_force_vec.x() = grid_force * cos(current_el) * cos(current_az);
            // y分量：水平侧向（方位角正弦 × 俯仰角余弦 × 合力大小）
            grid_force_vec.y() = grid_force * cos(current_el) * sin(current_az);
            // z分量：垂直方向（俯仰角正弦 × 合力大小）
            grid_force_vec.z() = grid_force * sin(current_el);

            // 4. 累加至总体合力矢量
            total_force += grid_force_vec;
        }
    }

    // 5. 处理总体合力：若合力接近零（无有效方向），默认指向目标方向
    const double force_threshold = 1e-6;
    if (total_force.norm() < force_threshold) {
        ROS_WARN("GenerateForceDir: Total force is near zero, use default goal direction");
        // 从目标极坐标参数（需提前存储为类成员）转换为默认方向
        total_force.x() = cos(goal_el_) * cos(goal_az_);
        total_force.y() = cos(goal_el_) * sin(goal_az_);
        total_force.z() = sin(goal_el_);
    }

    // 6. 归一化合力矢量，得到最终运动方向（单位向量）
    Eigen::Vector3d final_dir = total_force.normalized();

    // 7. 线程安全更新当前方向（加锁保护共享数据）
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_direction_ = final_dir;
    current_polar_field_.force_vector = total_force;  // 存储原始合力，用于后续调试或反馈
}