#include "mid360_avoidance/cost_calculator.h" 
#include "mid360_avoidance/utils.h" 
#include #include #include #include
namespace mid360_avoidance {
    
//构造函数
CostCalculator::CostCalculator() :
    histogram_updated_(false), is_updated_(false),safety_distance_(1.0), 
    max_sensor_range_(50.0),obstacle_weight_(0.5), goal_weight_(0.3),  
    smoothness_weight_(0.2),goal_azimuth_(0.0), goal_elevation_(0.0) {
}

//设置直方图
void CostCalculator::setHistogram(const PolarHistogram& histogram){
    
    std::lock_guardstd::mutex lock (data_mutex_);
    histogram_ = histogram;
    histogram_updated_ = true;
    
    // 同步成本矩阵与直方图的网格尺寸
    cost_matrix_.num_azimuth_bins = histogram.num_azimuth_bins;
    cost_matrix_.num_elevation_bins = histogram.num_elevation_bins;
    cost_matrix_.data.resize (histogram.num_azimuth_bins,std::vector<double>(histogram.num_elevation_bins, 0.0));
    
    // 更新最大感知距离（与直方图匹配）
    max_sensor_range_ = histogram_.data [0][0] == INFINITY ? 50.0 : max_sensor_range_;

}

//设置目标点
void CostCalculator::setGoal(const Eigen::Vector3d& goal) {
    
    std::lock_guardstd::mutex lock(data_mutex_);goal_ = goal;
    
    // 计算目标相对于机体坐标系的方向角（需结合当前位置）
    Eigen::Vector3d goal_dir = goal_ - robot_position_;
    
    // 目标过近时无需计算方向
    if (goal_dir.norm () < 1e-3) { 
        goal_azimuth_ = 0.0;goal_elevation_ = 0.0;
        return;
    }

    // 计算方位角（绕 z 轴，x 正方向为 0，逆时针为正）
    goal_azimuth_ = std::atan2 (goal_dir.y (), goal_dir.x ());
    goal_azimuth_ = normalizeAngle (goal_azimuth_); // 归一化到 [-π, π]

    // 计算仰角（与 xy 平面的夹角，向上为正）
    double horizontal_dist = std::sqrt (goal_dir.x ()*goal_dir.x () + goal_dir.y ()*goal_dir.y ());
    goal_elevation_ = std::atan2 (goal_dir.z (), horizontal_dist);
    
    // 裁剪仰角到直方图范围（防止目标超出传感器视场）
    goal_elevation_ = std::max (histogram_.min_elevation,std::min (histogram_.max_elevation, goal_elevation_));
}

//机体速度和位置
void CostCalculator::setRobotState(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) {
    std::lock_guardstd::mutex lock(data_mutex_);
    robot_position_ = position;
    robot_velocity_ = velocity;
}

//计算成本
bool CostCalculator::computeCostMatrix() {
   
    std::lock_guardstd::mutex lock(data_mutex_);

    // 检查输入数据有效性
    if (!histogram_updated_ || goal_.norm () < 1e-3) {
        ROS_DEBUG_THROTTLE (1.0, "Cost calculation skipped: invalid input (histogram/goal)");
        return false;
    }

    // 遍历所有角度网格，计算每个方向的成本
    for (size_t az = 0; az < cost_matrix_.num_azimuth_bins; ++az) {
        for (size_t el = 0; el < cost_matrix_.num_elevation_bins; ++el) {
            // 计算当前网格的中心角度
            double az_angle = histogram_.min_azimuth +(az + 0.5) * histogram_.azimuth_resolution;
            double el_angle = histogram_.min_elevation +(el + 0.5) * histogram_.elevation_resolution;

            // 计算单个方向的总成本c
            ost_matrix_.data [az][el] = calculateDirectionCost (az_angle, el_angle, az, el);
            // 确保成本在 0~1 范围内
            cost_matrix_.data [az][el] = std::max (0.0, std::min (1.0, cost_matrix_.data [az][el]));
        }
    }

    // 平滑成本矩阵，减少噪声导致的局部极小值
    smoothCostMatrix ();

    // 更新状态
    cost_matrix_.timestamp = ros::Time::now ();
    histogram_updated_ = false;
    {
        std::lock_guardstd::mutex update_lock(update_mutex_);
        is_updated_ = true;
    }

    ROS_DEBUG_THROTTLE(2.0, "Cost matrix computed successfully (size: %zu x %zu)",cost_matrix_.num_azimuth_bins, cost_matrix_.num_elevation_bins);
    return true;
}

//只读成本矩阵
const CostMatrix& CostCalculator::getCostMatrix() const {
    std::lock_guardstd::mutex lock(data_mutex_);
    return cost_matrix_;
}

//更新
bool CostCalculator::isUpdated() const {
    std::lock_guardstd::mutex lock(update_mutex_);
    return is_updated_;
}

//重置更新
void CostCalculator::resetUpdatedFlag() {
    std::lock_guardstd::mutex lock(update_mutex_);
    is_updated_ = false;
}

//权重设置
void CostCalculator::setParameters(double safety_distance, double obstacle_weight,
    double goal_weight, double smoothness_weight) 
    {
    std::lock_guardstd::mutex lock (data_mutex_);
    safety_distance_ = std::max (0.1, safety_distance); 
        
    // 安全距离最小0.1m
    obstacle_weight_ = std::max (0.0, std::min (1.0, obstacle_weight));
    goal_weight_ = std::max (0.0, std::min (1.0, goal_weight));
    smoothness_weight_ = std::max (0.0, std::min (1.0, smoothness_weight));

    // 确保权重和为 1（归一化）
    double total_weight = obstacle_weight_ + goal_weight_ + smoothness_weight_;
    if (total_weight < 1e-3) {
        obstacle_weight_ = 1.0;
        goal_weight_ = 0.0;
        smoothness_weight_ = 0.0;
        ROS_WARN ("Cost weights sum to zero, resetting to obstacle-only weight");
    } else {
        obstacle_weight_ /= total_weight;
        goal_weight_ /= total_weight;
        smoothness_weight_ /= total_weight;
    }

    ROS_INFO("Cost calculator parameters updated:");
    ROS_INFO(" safety_distance: %.2f m", safety_distance_);
    ROS_INFO(" weights (obstacle: %.2f, goal: %.2f, smoothness: %.2f)",obstacle_weight_, goal_weight_, smoothness_weight_);
}

//导航成本
double CostCalculator::calculateDirectionCost (double azimuth, double elevation,size_t bin_az, size_t bin_el) {
    
    // 1. 计算障碍物距离成本
    double obs_cost = obstacleDistanceCost (histogram_.data [bin_az][bin_el]);

    // 2. 计算目标方向偏差成本
    double goal_cost = goalDirectionCost (azimuth, elevation);

    // 3. 计算运动平滑性成本
    double smooth_cost = smoothnessCost (azimuth, elevation);

    // 加权融合总成本
    double total_cost = (obstacle_weight_ * obs_cost) +(goal_weight_ * goal_cost) +(smoothness_weight_ * smooth_cost);

    return total_cost;
}

//障碍物成本
double CostCalculator::obstacleDistanceCost (double distance) {
    if (distance == INFINITY || distance > max_sensor_range_) {
        // 无障碍物或超出感知范围，成本为 0
        return 0.0;
    }if (distance < safety_distance_) {
        // 小于安全距离，成本为 1（不可通行）
        return 1.0;
    }

    // 距离在安全距离～最大感知范围之间，成本线性递减
    double cost = 1.0 - ((distance - safety_distance_) / (max_sensor_range_ - safety_distance_));
    return std::max (0.0, std::min (1.0, cost));
}

//目标成本
double CostCalculator::goalDirectionCost (double azimuth, double elevation) {
    // 计算当前方向与目标方向的角度差（方位角 + 仰角）
    double az_diff = std::abs (normalizeAngle (azimuth - goal_azimuth_));
    double el_diff = std::abs (elevation - goal_elevation_);

    // 方位角最大差为 π（180°），仰角最大差为直方图仰角范围
    double max_az_diff = M_PI;double max_el_diff = histogram_.max_elevation - histogram_.min_elevation;

    // 归一化角度差到 0~1 范围
    double normalized_az_diff = az_diff /max_az_diff;
    double normalized_el_diff = max_el_diff <1e-3 ? 0.0 : (el_diff /max_el_diff);

    // 融合方位角和仰角偏差成本（权重各 0.5）
    double goal_cost = 0.5 * normalized_az_diff + 0.5 * normalized_el_diff;
    return std::max (0.0, std::min (1.0, goal_cost));
}

//平滑成本
double CostCalculator::smoothnessCost (double azimuth, double elevation) {
    // 若当前速度过小，平滑性成本为 0（无需考虑运动连续性）
    if (robot_velocity_.norm () < 0.1) {
        return 0.0;
    }

    // 当前速度方向的单位向量
    Eigen::Vector3d vel_dir = robot_velocity_.normalized ();

    // 当前规划方向的单位向量
    Eigen::Vector3d plan_dir = anglesToDirectionVector (azimuth, elevation);

    // 计算两个方向的夹角余弦（-1~1），转换为成本（0~1）
    double dot_product = std::max (-1.0, std::min (1.0, vel_dir.dot (plan_dir)));
    double angle_diff = std::acos (dot_product); // 0~πdouble 
    smooth_cost = angle_diff / M_PI; // 归一化到 0~1

    return std::max(0.0, std::min(1.0, smooth_cost));
}

//平滑成本矩阵
void CostCalculator::smoothCostMatrix () {
    // 3x3 滑动窗口平滑（忽略边界网格，避免越界）
    const int window_size = 3;
    const int half_window = window_size / 2;

    // 临时存储平滑前的成本矩阵
    std::vector<std::vector<double>> temp_data = cost_matrix_.data;

    for (size_t az = half_window; az < cost_matrix_.num_azimuth_bins - half_window; ++az) {
        for (size_t el = half_window; el < cost_matrix_.num_elevation_bins - half_window;++el) {
            // 计算窗口内的平均成本
            double sum = 0.0;
            int count = 0;
            for (int d_az = -half_window; d_az <= half_window; ++d_az) {
                for (int d_el = -half_window; d_el <= half_window; ++d_el) {
                    size_t neighbor_az = az + d_az;
                    size_t neighbor_el = el + d_el;
                    // 确保邻居索引有效
                    if (neighbor_az < cost_matrix_.num_azimuth_bins &&neighbor_el < cost_matrix_.num_elevation_bins) {
                        sum += temp_data [neighbor_az][neighbor_el];
                        count++;
                    }
                }
            }
            // 更新平滑后的成本
            if (count > 0) {
                cost_matrix_.data [az][el] = sum /count;
            }
        }
    }
}

//？
Eigen::Vector3d CostCalculator::anglesToDirectionVector (double azimuth, double elevation) {
    Eigen::Vector3d dir;
    // 极坐标转笛卡尔坐标（机体坐标系：x 前，y 右，z 上）
    double horizontal_dist = std::cos (elevation);
    dir.x () = horizontal_dist * std::cos (azimuth);
    dir.y () = horizontal_dist * std::sin (azimuth);
    dir.z () = std::sin (elevation);return dir.normalized ();
}

} // namespace mid360_avoidance