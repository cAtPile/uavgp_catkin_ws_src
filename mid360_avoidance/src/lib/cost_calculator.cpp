#include "mid360_avoidance/cost_calculator.h" 
#include "mid360_avoidance/utils.h" 
#include <mutex>          // 用于线程同步的互斥锁
#include <ros/ros.h>      // ROS相关功能
#include <Eigen/Dense>    // Eigen矩阵矩阵运算
#include <cmath>          // 数学函数

namespace mid360_avoidance {
    
// 构造函数：初始化成本计算器参数
CostCalculator::CostCalculator() :
    histogram_updated_(false),  // 直方图未更新
    is_updated_(false),         // 成本矩阵未更新
    safety_distance_(1.0),      // 安全距离默认1.0米
    max_sensor_range_(50.0),    // 最大感知距离默认50米
    obstacle_weight_(0.5),      // 障碍物成本权重
    goal_weight_(0.3),          // 目标方向成本权重
    smoothness_weight_(0.2),    // 运动平滑性成本权重
    goal_azimuth_(0.0),         // 目标方位角初始化为0
    goal_elevation_(0.0) {      // 目标仰角初始化为0
}

// 设置极坐标直方图数据
void CostCalculator::setHistogram(const PolarHistogram& histogram){
    
    // 加锁保证线程安全，防止数据竞争
    std::lock_guard<std::mutex> lock(data_mutex_);
    histogram_ = histogram;              // 保存直方图数据
    histogram_updated_ = true;           // 标记直方图已更新
    
    // 同步成本矩阵与直方图的网格尺寸
    cost_matrix_.num_azimuth_bins = histogram.num_azimuth_bins;
    cost_matrix_.num_elevation_bins = histogram.num_elevation_bins;
    cost_matrix_.data.resize(histogram.num_azimuth_bins, 
                           std::vector<double>(histogram.num_elevation_bins, 0.0));
    
    // 更新最大感知距离（与直方图匹配）
    if (histogram_.data[0][0] != INFINITY) {
        max_sensor_range_ = histogram_.max_range;  // 使用直方图中的最大距离
    }
}

// 设置目标点坐标（机体坐标系）
void CostCalculator::setGoal(const Eigen::Vector3d& goal) {
    // 加锁保证线程安全
    std::lock_guard<std::mutex> lock(data_mutex_);
    goal_ = goal;  // 保存目标点坐标
    
    // 计算目标相对于机体坐标系的方向向量（目标点 - 机器人位置）
    Eigen::Vector3d goal_dir = goal_ - robot_position_;
    
    // 目标点过近（距离小于0.001米）时无需计算方向
    if (goal_dir.norm() < 1e-3) { 
        goal_azimuth_ = 0.0;
        goal_elevation_ = 0.0;
        return;
    }

    // 计算方位角（绕z轴，x正方向为0，逆时针为正）范围[-π, π]
    goal_azimuth_ = std::atan2(goal_dir.y(), goal_dir.x());
    goal_azimuth_ = normalizeAngle(goal_azimuth_);  // 归一化角度到[-π, π]

    // 计算仰角（与xy平面的夹角，向上为正）范围[-π/2, π/2]
    double horizontal_dist = std::sqrt(goal_dir.x()*goal_dir.x() + goal_dir.y()*goal_dir.y());
    goal_elevation_ = std::atan2(goal_dir.z(), horizontal_dist);
    
    // 裁剪仰角到直方图的有效范围（防止目标超出传感器视场）
    goal_elevation_ = std::max(histogram_.min_elevation,
                              std::min(histogram_.max_elevation, goal_elevation_));
}

// 设置机器人位置和速度（用于平滑性成本计算）
void CostCalculator::setRobotState(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    robot_position_ = position;  // 保存机器人位置
    robot_velocity_ = velocity;  // 保存机器人速度
}

// 计算成本矩阵：遍历所有角度网格，计算每个方向的总成本
bool CostCalculator::computeCostMatrix() {
    // 加锁保证线程安全
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 检查输入数据有效性：直方图未更新或目标点无效时返回失败
    if (!histogram_updated_ || goal_.norm() < 1e-3) {
        ROS_DEBUG_THROTTLE(1.0, "Cost calculation skipped: invalid input (histogram/goal)");
        return false;
    }

    // 遍历所有角度网格，计算每个方向的成本
    for (size_t az = 0; az < cost_matrix_.num_azimuth_bins; ++az) {
        for (size_t el = 0; el < cost_matrix_.num_elevation_bins; ++el) {
            // 计算当前网格的中心角度（加上0.5*分辨率得到网格中心）
            double az_angle = histogram_.min_azimuth + (az + 0.5) * histogram_.azimuth_resolution;
            double el_angle = histogram_.min_elevation + (el + 0.5) * histogram_.elevation_resolution;

            // 计算单个方向的总成本
            cost_matrix_.data[az][el] = calculateDirectionCost(az_angle, el_angle, az, el);
            // 确保成本在0~1范围内（归一化）
            cost_matrix_.data[az][el] = std::max(0.0, std::min(1.0, cost_matrix_.data[az][el]));
        }
    }

    // 平滑成本矩阵，减少噪声导致的局部极小值
    smoothCostMatrix();

    // 更新状态信息
    cost_matrix_.timestamp = ros::Time::now();  // 记录计算时间戳
    histogram_updated_ = false;                 // 重置直方图更新标志
    {
        std::lock_guard<std::mutex> update_lock(update_mutex_);
        is_updated_ = true;                     // 标记成本矩阵已更新
    }

    ROS_DEBUG_THROTTLE(2.0, "Cost matrix computed successfully (size: %zu x %zu)",
                      cost_matrix_.num_azimuth_bins, cost_matrix_.num_elevation_bins);
    return true;
}

// 获取成本矩阵（只读，线程安全）
const CostMatrix& CostCalculator::getCostMatrix() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return cost_matrix_;
}

// 检查成本矩阵是否已更新
bool CostCalculator::isUpdated() const {
    std::lock_guard<std::mutex> lock(update_mutex_);
    return is_updated_;
}

// 重置成本矩阵更新标志
void CostCalculator::resetUpdatedFlag() {
    std::lock_guard<std::mutex> lock(update_mutex_);
    is_updated_ = false;
}

// 设置成本计算参数（权重和安全距离）
void CostCalculator::setParameters(double safety_distance, double obstacle_weight,
                                 double goal_weight, double smoothness_weight) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    // 安全距离最小为0.1米（防止过小值）
    safety_distance_ = std::max(0.1, safety_distance); 
    
    // 权重限制在0~1范围内
    obstacle_weight_ = std::max(0.0, std::min(1.0, obstacle_weight));
    goal_weight_ = std::max(0.0, std::min(1.0, goal_weight));
    smoothness_weight_ = std::max(0.0, std::min(1.0, smoothness_weight));

    // 确保权重和为1（归一化处理）
    double total_weight = obstacle_weight_ + goal_weight_ + smoothness_weight_;
    if (total_weight < 1e-3) {  // 权重和接近0时重置为默认值
        obstacle_weight_ = 1.0;
        goal_weight_ = 0.0;
        smoothness_weight_ = 0.0;
        ROS_WARN("Cost weights sum to zero, resetting to obstacle-only weight");
    } else {  // 归一化权重
        obstacle_weight_ /= total_weight;
        goal_weight_ /= total_weight;
        smoothness_weight_ /= total_weight;
    }

    ROS_INFO("Cost calculator parameters updated:");
    ROS_INFO(" safety_distance: %.2f m", safety_distance_);
    ROS_INFO(" weights (obstacle: %.2f, goal: %.2f, smoothness: %.2f)",
            obstacle_weight_, goal_weight_, smoothness_weight_);
}

// 计算单个方向（角度）的总成本
double CostCalculator::calculateDirectionCost(double azimuth, double elevation, 
                                            size_t bin_az, size_t bin_el) {
    // 1. 计算障碍物距离成本（近距离成本高）
    double obs_cost = obstacleDistanceCost(histogram_.data[bin_az][bin_el]);

    // 2. 计算目标方向偏差成本（偏离目标方向成本高）
    double goal_cost = goalDirectionCost(azimuth, elevation);

    // 3. 计算运动平滑性成本（与当前速度方向夹角大则成本高）
    double smooth_cost = smoothnessCost(azimuth, elevation);

    // 加权融合总成本
    double total_cost = (obstacle_weight_ * obs_cost) + 
                       (goal_weight_ * goal_cost) + 
                       (smoothness_weight_ * smooth_cost);

    return total_cost;
}

// 计算障碍物距离成本
double CostCalculator::obstacleDistanceCost(double distance) {
    if (distance == INFINITY || distance > max_sensor_range_) {
        // 无障碍物或超出感知范围，成本为0（可通行）
        return 0.0;
    }
    if (distance < safety_distance_) {
        // 小于安全距离，成本为1（不可通行）
        return 1.0;
    }

    // 距离在安全距离～最大感知范围之间，成本线性递减
    // 距离越近成本越高，距离越远成本越低
    double cost = 1.0 - ((distance - safety_distance_) / (max_sensor_range_ - safety_distance_));
    return std::max(0.0, std::min(1.0, cost));  // 确保成本在0~1之间
}

// 计算目标方向偏差成本
double CostCalculator::goalDirectionCost(double azimuth, double elevation) {
    // 计算当前方向与目标方向的角度差（方位角 + 仰角）
    double az_diff = std::abs(normalizeAngle(azimuth - goal_azimuth_));  // 归一化角度差
    double el_diff = std::abs(elevation - goal_elevation_);

    // 确定最大角度差（用于归一化）
    double max_az_diff = M_PI;  // 方位角最大差为π（180°）
    double max_el_diff = histogram_.max_elevation - histogram_.min_elevation;  // 仰角最大范围

    // 归一化角度差到0~1范围
    double normalized_az_diff = az_diff / max_az_diff;
    double normalized_el_diff = (max_el_diff < 1e-3) ? 0.0 : (el_diff / max_el_diff);

    // 融合方位角和仰角偏差成本（权重各0.5）
    double goal_cost = 0.5 * normalized_az_diff + 0.5 * normalized_el_diff;
    return std::max(0.0, std::min(1.0, goal_cost));  // 确保成本在0~1之间
}

// 计算运动平滑性成本（与当前速度方向的一致性）
double CostCalculator::smoothnessCost(double azimuth, double elevation) {
    // 若当前速度过小（小于0.1m/s），平滑性成本为0（无需考虑运动连续性）
    if (robot_velocity_.norm() < 0.1) {
        return 0.0;
    }

    // 当前速度方向的单位向量
    Eigen::Vector3d vel_dir = robot_velocity_.normalized();

    // 当前规划方向的单位向量（从角度转换为三维向量）
    Eigen::Vector3d plan_dir = anglesToDirectionVector(azimuth, elevation);

    // 计算两个方向的夹角余弦（范围-1~1），转换为角度差（0~π）
    double dot_product = std::max(-1.0, std::min(1.0, vel_dir.dot(plan_dir)));
    double angle_diff = std::acos(dot_product);  // 角度差（弧度）
    
    // 归一化到0~1范围（角度差越大，成本越高）
    double smooth_cost = angle_diff / M_PI;
    return std::max(0.0, std::min(1.0, smooth_cost));
}

// 平滑成本矩阵（减少噪声影响）
void CostCalculator::smoothCostMatrix() {
    // 3x3滑动窗口平滑（忽略边界网格，避免越界）
    const int window_size = 3;
    const int half_window = window_size / 2;

    // 临时存储平滑前的成本矩阵（避免边算边改导致的误差）
    std::vector<std::vector<double>> temp_data = cost_matrix_.data;

    // 遍历所有非边界网格
    for (size_t az = half_window; az < cost_matrix_.num_azimuth_bins - half_window; ++az) {
        for (size_t el = half_window; el < cost_matrix_.num_elevation_bins - half_window; ++el) {
            // 计算3x3窗口内的平均成本
            double sum = 0.0;
            int count = 0;
            for (int d_az = -half_window; d_az <= half_window; ++d_az) {
                for (int d_el = -half_window; d_el <= half_window; ++d_el) {
                    size_t neighbor_az = az + d_az;
                    size_t neighbor_el = el + d_el;
                    // 确保邻居索引有效
                    if (neighbor_az < cost_matrix_.num_azimuth_bins && 
                        neighbor_el < cost_matrix_.num_elevation_bins) {
                        sum += temp_data[neighbor_az][neighbor_el];
                        count++;
                    }
                }
            }
            // 更新平滑后的成本（窗口内平均值）
            if (count > 0) {
                cost_matrix_.data[az][el] = sum / count;
            }
        }
    }
}

// 将方位角和仰角转换为三维方向向量（单位向量）
Eigen::Vector3d CostCalculator::anglesToDirectionVector(double azimuth, double elevation) {
    Eigen::Vector3d dir;
    // 极坐标转笛卡尔坐标（机体坐标系：x前，y右，z上）
    double horizontal_dist = std::cos(elevation);  // 水平距离分量（仰角影响）
    dir.x() = horizontal_dist * std::cos(azimuth);  // x分量（前向）
    dir.y() = horizontal_dist * std::sin(azimuth);  // y分量（右向）
    dir.z() = std::sin(elevation);                  // z分量（上向）
    return dir.normalized();  // 归一化为单位向量
}

} // namespace mid360_avoidance