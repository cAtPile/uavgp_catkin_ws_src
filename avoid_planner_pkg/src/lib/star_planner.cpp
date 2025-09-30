#include "avoid_planner_pkg/star_planner.h"
#include "avoid_planner_pkg/utils.h"
#include <cmath>
#include <queue>
#include <algorithm>
#include <ros/console.h>

namespace mid360_avoidance {

/**
 * @brief 节点优先级比较器结构体（用于优先队列排序）
 * @details 定义星形搜索中节点的优先级判定规则，确保成本更低的节点优先被探索
 * 注：该结构体需配合搜索树使用，通过节点索引计算总成本
 */
struct NodePriorityCompare {
    /**
     * @brief 重载()运算符，实现节点优先级比较逻辑
     * @param a_idx 节点a在搜索树中的索引
     * @param b_idx 节点b在搜索树中的索引
     * @param search_tree 存储所有搜索节点的向量（含节点成本、启发式值等信息）
     * @return true表示节点a总成本高于节点b（a优先级更低，应排在队列后面），false相反
     */
    bool operator()(const size_t& a_idx, const size_t& b_idx, 
                   const std::vector<StarNode>& search_tree) {
        // 总成本计算：累计成本（已行驶路径的实际成本） + 启发式成本（预估剩余成本）× 节点深度
        // 深度加权可避免过度偏向短期低成本路径，平衡全局搜索效率
        double cost_a = search_tree[a_idx].cost + 
                       search_tree[a_idx].heuristic * search_tree[a_idx].depth;
        double cost_b = search_tree[b_idx].cost + 
                       search_tree[b_idx].heuristic * search_tree[b_idx].depth;
        return cost_a > cost_b;  // 优先队列默认大顶堆，通过>实现"小成本节点优先出队"
    }
};

/**
 * @brief 星形规划器构造函数（初始化默认参数）
 * @details 初始化规划器核心参数、角度范围和状态标志，默认值适配低空激光雷达避障场景
 * 关键参数说明：
 * - max_depth_: 搜索树最大深度（防止深度过深导致计算量激增）
 * - min_elevation_/max_elevation_: 仰角范围（-15°~15°，弧度制0.2618≈15°）
 * - has_valid_cost_matrix_: 成本矩阵有效性标志（初始为false，需外部设置后生效）
 */
StarPlanner::StarPlanner() : 
    has_valid_cost_matrix_(false),  // 初始无有效成本矩阵
    max_depth_(5),                  // 搜索树最大深度（默认5层）
    max_nodes_(50),                 // 最大探索节点数（默认50个，控制计算资源占用）
    step_size_(0.5),                // 每个规划步长（默认0.5米，平衡精度与效率）
    goal_tolerance_(0.3),           // 目标到达容忍度（默认0.3米，允许微小偏差）
    heuristic_weight_(1.0),         // 启发式成本权重（默认1.0，平衡探索与目标导向）
    min_azimuth_(-M_PI),            // 最小方位角（-π，对应180°，机体x轴负方向）
    max_azimuth_(M_PI),             // 最大方位角（π，对应180°，机体x轴正方向）
    min_elevation_(-0.2618),        // 最小仰角（-0.2618rad≈-15°，避免过低飞行）
    max_elevation_(0.2618) {}       // 最大仰角（0.2618rad≈15°，避免过高飞行）

/**
 * @brief 设置成本矩阵（从成本计算器模块传入）
 * @details 接收外部计算的成本矩阵（含障碍物、目标方向、运动平滑性等综合成本）
 * 是路径规划的核心输入，决定各方向的通行成本高低
 * @param cost_matrix 外部传入的CostMatrix类型数据（网格化的方向成本）
 */
void StarPlanner::setCostMatrix(const CostMatrix& cost_matrix) {
    std::lock_guard<std::mutex> lock(result_mutex_);  // 加锁保证线程安全（防止规划中修改成本矩阵）
    cost_matrix_ = cost_matrix;                       // 保存成本矩阵到成员变量
    has_valid_cost_matrix_ = true;                    // 标记成本矩阵已有效
}

/**
 * @brief 设置规划的起点和目标点（机体坐标系下）
 * @details 定义路径规划的起始位置和终止位置，均为三维坐标（x,y,z）
 * 注：坐标需基于统一的机体坐标系，确保方向计算准确性
 * @param start 起点坐标（如机器人当前位置）
 * @param goal 目标点坐标（如期望到达的下一位置）
 */
void StarPlanner::setStartAndGoal(const Eigen::Vector3d& start, const Eigen::Vector3d& goal) {
    std::lock_guard<std::mutex> lock(result_mutex_);  // 加锁保证线程安全（防止多线程读写冲突）
    start_position_ = start;  // 保存起点坐标
    goal_position_ = goal;    // 保存目标点坐标
}

/**
 * @brief 设置机器人当前速度（用于初始方向计算）
 * @details 规划的初始方向优先跟随当前速度方向，避免运动方向突变，保证行驶平稳性
 * 若速度过小（ norm < 1e-3 ），则默认指向目标点方向
 * @param velocity 机器人当前三维速度向量（机体坐标系下）
 */
void StarPlanner::setCurrentVelocity(const Eigen::Vector3d& velocity) {
    std::lock_guard<std::mutex> lock(result_mutex_);  // 加锁保证线程安全
    current_velocity_ = velocity;  // 保存当前速度到成员变量
}

/**
 * @brief 核心函数：执行星形搜索生成避障路径
 * @details 基于成本矩阵、起点/目标和当前速度，通过优先级队列探索最优路径，
 * 包含输入有效性检查、搜索树初始化、节点扩展、路径回溯与平滑等完整流程
 * @return PathResult 规划结果结构体（含路径点、成功标志、总成本、时间戳）
 */
PathResult StarPlanner::searchPath() {
    std::lock_guard<std::mutex> lock(result_mutex_);  // 加锁保证线程安全（防止多线程同时执行规划）
    PathResult result;                                // 初始化规划结果（默认success=false）
    result.timestamp = ros::Time::now();              // 记录规划开始时间戳
    
    // 1. 输入有效性检查（成本矩阵、起点、目标均需有效，否则规划失败）
    if (!has_valid_cost_matrix_ ||          // 无有效成本矩阵
        start_position_.isZero() ||         // 起点未设置（全零向量视为无效）
        goal_position_.isZero() ||          // 目标点未设置
        cost_matrix_.data.empty()) {        // 成本矩阵数据为空（无方向成本信息）
        ROS_WARN_THROTTLE(1.0, "Invalid input for star search");  // 1秒内仅输出一次警告（避免刷屏）
        return result;  // 返回空结果（success默认false）
    }
    
    // 2. 初始化搜索树（存储所有探索过的节点，每个节点含位置、方向、成本等信息）
    search_tree_.clear();  // 清空历史搜索树（避免残留数据影响）
    // 计算初始方向：优先跟随当前速度，速度过小时指向目标
    Eigen::Vector3d initial_direction = current_velocity_.norm() > 1e-3 ? 
                                       current_velocity_.normalized() :  // 速度有效→沿速度方向
                                       (goal_position_ - start_position_).normalized();  // 速度无效→指向目标
    // 创建根节点（搜索树的起点，对应机器人当前位置）
    StarNode root_node(
        start_position_,       // 节点位置（起点坐标）
        initial_direction,     // 节点方向（初始方向）
        0.0,                   // 累计成本（起点无成本，设为0）
        calculateHeuristic(start_position_),  // 启发式成本（起点到目标的预估成本）
        -1,                    // 父节点索引（根节点无父节点，设为-1）
        0                      // 节点深度（根节点深度为0）
    );
    search_tree_.push_back(root_node);  // 将根节点加入搜索树
    
    // 3. 初始化优先级队列（按节点总成本排序，小成本节点优先探索）
    // 自定义比较函数，引用当前规划器的搜索树和启发式权重（确保捕获this指针）
    std::priority_queue<size_t, std::vector<size_t>, 
                       std::function<bool(size_t, size_t)>> 
        priority_queue(
            [this](size_t a, size_t b) {
                // 总成本 = 累计成本 + 启发式成本 × 权重（平衡探索与目标导向）
                double cost_a = search_tree_[a].cost + 
                               search_tree_[a].heuristic * heuristic_weight_;
                double cost_b = search_tree_[b].cost + 
                               search_tree_[b].heuristic * heuristic_weight_;
                return cost_a > cost_b;  // 优先队列大顶堆特性→实现"小成本优先出队"
            }
        );
    
    // 4. 初始化已访问节点集合（避免重复探索同一节点，减少冗余计算）
    std::set<size_t> visited_nodes;
    
    // 5. 启动搜索：将根节点加入队列和已访问集合
    priority_queue.push(0);          // 根节点在搜索树中索引为0，加入优先队列
    visited_nodes.insert(0);         // 标记根节点为已访问
    
    // 6. 目标节点相关变量初始化
    size_t goal_node_index = -1;     // 目标节点在搜索树中的索引（初始为无效值-1）
    bool goal_found = false;         // 是否找到目标的标志（初始为false）
    
    // 7. 星形搜索主循环（核心逻辑：探索节点→生成子节点→检查目标）
    while (!priority_queue.empty() &&        // 队列非空（仍有节点待探索）
           search_tree_.size() < max_nodes_ &&  // 未超过最大节点数（控制计算量）
           !goal_found) {                     // 未找到目标（找到则退出循环）
        
        // 7.1 取出当前成本最小的节点（优先队列顶元素）
        size_t current_idx = priority_queue.top();
        priority_queue.pop();                // 弹出队列（后续不再处理该节点的入队）
        StarNode current_node = search_tree_[current_idx];  // 获取当前节点的完整数据
        
        // 7.2 检查当前节点是否到达目标（距离≤目标容忍度则视为到达）
        if (isreachGoal(current_node.position)) {
            goal_node_index = current_idx;   // 记录目标节点在搜索树中的索引
            goal_found = true;               // 标记找到目标
            break;                           // 退出搜索循环（无需继续探索）
        }
        
        // 7.3 检查当前节点是否超过最大深度（防止深度过深导致计算量失控）
        if (current_node.depth >= max_depth_) {
            continue;  // 跳过该节点的子节点生成（不扩展深度超限节点）
        }
        
        // 7.4 生成当前节点的子节点方向（基于父节点方向的锥形采样，保证运动连续性）
        std::vector<Eigen::Vector3d> child_directions = 
            generateChildDirections(current_node.direction);
        
        // 7.5 遍历所有子方向，生成并处理子节点
        for (const auto& dir : child_directions) {
            // 7.5.1 计算子节点位置（父节点位置 + 方向向量 × 步长）
            Eigen::Vector3d child_pos = current_node.position + dir * step_size_;
            
            // 7.5.2 将子节点方向转换为成本矩阵的网格索引（用于获取该方向的成本）
            size_t az_index, el_index;  // 输出参数：方位角索引、仰角索引
            if (!directionToCostIndices(dir, az_index, el_index)) {
                continue;  // 方向超出成本矩阵的有效角度范围→跳过该子节点
            }
            
            // 7.5.3 获取该方向的成本值（从成本矩阵中读取，成本≥1.0视为不可通行）
            double direction_cost = cost_matrix_.data[az_index][el_index];
            if (direction_cost >= 1.0) {
                continue;  // 方向成本过高（接近障碍物）→跳过该子节点
            }
            
            // 7.5.4 计算子节点的累计成本（父节点成本 + 方向成本 × 步长）
            // 步长加权：确保长距离路径的成本计算更准确
            double child_cost = current_node.cost + direction_cost * step_size_;
            
            // 7.5.5 计算子节点的启发式成本（子节点到目标的预估成本）
            double child_heuristic = calculateHeuristic(child_pos);
            
            // 7.5.6 创建子节点（深度=父节点深度+1，父索引=当前节点索引）
            StarNode child_node(
                child_pos,        // 子节点位置（7.5.1计算结果）
                dir,              // 子节点方向（当前遍历的子方向）
                child_cost,       // 子节点累计成本（7.5.4计算结果）
                child_heuristic,  // 子节点启发式成本（7.5.5计算结果）
                current_idx,      // 子节点父索引（当前节点索引）
                current_node.depth + 1  // 子节点深度（父节点深度+1）
            );
            
            // 7.5.7 将子节点加入搜索树（新节点索引=当前搜索树大小）
            size_t child_idx = search_tree_.size();
            search_tree_.push_back(child_node);
            
            // 7.5.8 将未访问的子节点加入优先队列（避免重复探索）
            if (visited_nodes.find(child_idx) == visited_nodes.end()) {
                priority_queue.push(child_idx);      // 加入优先队列待探索
                visited_nodes.insert(child_idx);     // 标记为已访问
            }
            
            // 7.5.9 检查子节点是否到达目标（提前终止搜索，提升效率）
            if (isreachGoal(child_pos)) {
                goal_node_index = child_idx;   // 记录目标节点索引
                goal_found = true;             // 标记找到目标
                break;                         // 退出子方向循环（无需处理后续方向）
            }
        }
    }
    
    // 8. 处理搜索结果（生成最终路径或返回失败）
    if (goal_found && goal_node_index != -1) {
        // 8.1 回溯路径：从目标节点反向追溯到根节点（获取的是"目标→起点"的逆序路径）
        std::vector<Eigen::Vector3d> raw_path = backtrackPath(goal_node_index);
        
        // 8.2 平滑路径：减少原始路径的抖动（采用滑动平均算法，保留起点和终点）
        std::vector<Eigen::Vector3d> smoothed_path = smoothPath(raw_path);
        
        // 8.3 填充规划结果（标记成功，设置路径点、总成本和时间戳）
        result.waypoints = smoothed_path;    // 最终路径点（起点→目标的有序向量）
        result.success = true;               // 规划成功标志设为true
        result.total_cost = search_tree_[goal_node_index].cost;  // 路径总成本（目标节点的累计成本）
        result.timestamp = ros::Time::now(); // 更新时间戳为规划完成时间
        
        // 输出调试信息（路径长度和总成本，仅在ROS_DEBUG级别可见）
        ROS_DEBUG("Star search succeeded. Path length: %zu waypoints, Total cost: %.2f",
                 smoothed_path.size(), result.total_cost);
    } else {
        // 8.4 规划失败处理（未找到目标或超出计算限制）
        ROS_WARN_THROTTLE(1.0, "Star search failed. Nodes explored: %zu", search_tree_.size());
    }
    
    // 9. 保存最后一次规划结果（供外部模块如控制节点查询）
    last_path_ = result;
    return result;
}

// 设置星形规划器参数
void StarPlanner::setParameters(size_t max_depth, size_t max_nodes, double step_size,
                               double goal_tolerance, double heuristic_weight) {
    std::lock_guard<std::mutex> lock(result_mutex_);  // 线程安全锁，防止参数修改冲突
    max_depth_ = std::max(1ul, max_depth);            // 搜索树最大深度（至少为1）
    max_nodes_ = std::max(10ul, max_nodes);           // 最大探索节点数（至少为10）
    step_size_ = std::max(0.1, step_size);            // 规划步长（至少0.1米）
    goal_tolerance_ = std::max(0.1, goal_tolerance);  // 目标点到达容差（至少0.1米）
    heuristic_weight_ = std::max(0.1, heuristic_weight);  // 启发式权重（至少0.1）
    
    // 输出参数更新信息
    ROS_INFO("星形规划器参数已更新:");
    ROS_INFO("  最大深度: %zu, 最大节点数: %zu", max_depth_, max_nodes_);
    ROS_INFO("  步长: %.2f 米, 目标容差: %.2f 米", step_size_, goal_tolerance_);
    ROS_INFO("  启发式权重: %.2f", heuristic_weight_);
}

// 获取最新规划路径
const PathResult& StarPlanner::getLastPath() const {
    std::lock_guard<std::mutex> lock(result_mutex_);  // 线程安全锁
    return last_path_;  // 返回最近一次规划的路径结果
}

// 计算启发式成本（预估成本）
double StarPlanner::calculateHeuristic(const Eigen::Vector3d& node_position) {
    // 使用节点到目标点的欧氏距离作为启发式成本
    return calculateDistance(node_position, goal_position_);
}

// 将方向向量转换为成本矩阵的索引
bool StarPlanner::directionToCostIndices(const Eigen::Vector3d& direction,
                                        size_t& az_index, size_t& el_index) {
    // 计算方向向量对应的方位角和仰角
    double azimuth, elevation;
    directionToAngles(direction, azimuth, elevation);
    
    // 检查角度是否在有效范围内
    if (azimuth < min_azimuth_ || azimuth > max_azimuth_ ||
        elevation < min_elevation_ || elevation > max_elevation_) {
        return false;  // 角度超出范围，返回失败
    }
    
    // 计算角度分辨率（每个网格对应的角度大小）
    double az_resolution = (max_azimuth_ - min_azimuth_) / cost_matrix_.num_azimuth_bins;
    double el_resolution = (max_elevation_ - min_elevation_) / cost_matrix_.num_elevation_bins;
    
    // 将角度转换为成本矩阵中的网格索引
    az_index = static_cast<size_t>((azimuth - min_azimuth_) / az_resolution);
    el_index = static_cast<size_t>((elevation - min_elevation_) / el_resolution);
    
    // 边界检查，确保索引不超出矩阵范围
    if (az_index >= cost_matrix_.num_azimuth_bins) {
        az_index = cost_matrix_.num_azimuth_bins - 1;
    }
    if (el_index >= cost_matrix_.num_elevation_bins) {
        el_index = cost_matrix_.num_elevation_bins - 1;
    }
    
    return true;  // 转换成功
}

// 将方向向量转换为方位角和仰角
void StarPlanner::directionToAngles(const Eigen::Vector3d& direction,
                                   double& azimuth, double& elevation) {
    // 归一化方向向量（确保是单位向量）
    Eigen::Vector3d dir = direction.normalized();
    
    // 计算方位角（绕z轴，x轴正方向为0度，逆时针为正）
    azimuth = std::atan2(dir.y(), dir.x());
    azimuth = normalizeAngle(azimuth);  // 将角度归一化到[-π, π]范围
    
    // 计算仰角（与xy平面的夹角，向上为正）
    double horizontal_dist = std::sqrt(dir.x() * dir.x() + dir.y() * dir.y());
    if (horizontal_dist < 1e-6) {  // 水平距离接近0（正上或正下方向）
        elevation = dir.z() > 0 ? M_PI_2 : -M_PI_2;  // 正上为π/2，正下为-π/2
    } else {
        elevation = std::atan2(dir.z(), horizontal_dist);  // 常规情况计算仰角
    }
}

// 生成子节点的可能方向（基于父节点方向）
std::vector<Eigen::Vector3d> StarPlanner::generateChildDirections(const Eigen::Vector3d& parent_direction) {
    std::vector<Eigen::Vector3d> directions;  // 存储生成的方向向量
    
    // 以父节点方向为中心，在锥形范围内采样可能的子方向（保证运动连续性）
    const int num_azimuth_steps = 8;    // 方位角方向采样数量
    const int num_elevation_steps = 3;  // 仰角方向采样数量
    const double max_azimuth_delta = 0.5236;  // 最大方位角偏差（约30度，弧度制）
    const double max_elevation_delta = 0.2618; // 最大仰角偏差（约15度，弧度制）
    
    // 先将父节点方向转换为角度表示
    double parent_az, parent_el;
    directionToAngles(parent_direction, parent_az, parent_el);
    
    // 在方位角和仰角范围内生成采样点
    for (int i = 0; i < num_azimuth_steps; ++i) {
        for (int j = 0; j < num_elevation_steps; ++j) {
            // 计算相对角度偏差（中心位置偏差为0）
            double az_delta = max_azimuth_delta * (i - (num_azimuth_steps - 1)/2.0) / 
                            ((num_azimuth_steps - 1)/2.0);
            double el_delta = max_elevation_delta * (j - (num_elevation_steps - 1)/2.0) / 
                            ((num_elevation_steps - 1)/2.0);
            
            // 计算绝对角度（父节点角度 + 偏差）
            double az = parent_az + az_delta;
            double el = parent_el + el_delta;
            
            // 归一化角度到有效范围
            az = normalizeAngle(az);  // 方位角归一化到[-π, π]
            el = std::max(min_elevation_, std::min(max_elevation_, el));  // 仰角限制在有效范围
            
            // 将角度转换为三维方向向量
            double horizontal = std::cos(el);  // 水平分量（受仰角影响）
            Eigen::Vector3d dir(
                horizontal * std::cos(az),  // x分量（前向）
                horizontal * std::sin(az),  // y分量（右向）
                std::sin(el)                // z分量（上向）
            );
            
            directions.push_back(dir.normalized());  // 归一化后加入方向列表
        }
    }
    
    // 去除重复方向（避免冗余计算）
    // 1. 排序方向向量（为去重做准备）
    std::sort(directions.begin(), directions.end(),
             [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                 return a.x() < b.x() || 
                        (a.x() == b.x() && a.y() < b.y()) ||
                        (a.x() == b.x() && a.y() == b.y() && a.z() < b.z());
             });
    // 2. 去重（向量差的模长小于0.001视为相同方向）
    auto last = std::unique(directions.begin(), directions.end(),
                           [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                               return (a - b).norm() < 1e-3;
                           });
    directions.erase(last, directions.end());  // 删除重复元素
    
    return directions;  // 返回去重后的方向列表
}

// 回溯路径（从目标节点到起点）
std::vector<Eigen::Vector3d> StarPlanner::backtrackPath(size_t goal_node_index) {
    std::vector<Eigen::Vector3d> path;  // 存储路径点
    
    // 从目标节点开始，沿父节点索引回溯到根节点
    size_t current_idx = goal_node_index;
    while (current_idx != -1 && current_idx < search_tree_.size()) {
        path.push_back(search_tree_[current_idx].position);  // 添加当前节点位置
        current_idx = search_tree_[current_idx].parent_index;  // 移动到父节点
    }
    
    // 反转路径，得到从起点到目标的正确顺序
    std::reverse(path.begin(), path.end());
    
    return path;  // 返回有序路径
}

// 检查是否到达目标点
bool StarPlanner::isreachGoal(const Eigen::Vector3d& node_position) {
    // 计算节点位置到目标点的距离
    double distance = calculateDistance(node_position, goal_position_);
    // 距离小于等于目标容差时视为到达
    return distance <= goal_tolerance_;
}

// 平滑路径（减少抖动）
std::vector<Eigen::Vector3d> StarPlanner::smoothPath(const std::vector<Eigen::Vector3d>& path) {
    if (path.size() <= 2) {
        return path;  // 路径点数量小于等于2时，无需平滑
    }
    
    // 使用滑动平均算法平滑路径
    std::vector<Eigen::Vector3d> smoothed = path;  // 初始化平滑路径为原始路径
    const size_t window_size = 3;  // 滑动窗口大小（3个点）
    
    // 只平滑中间点，保留起点和终点
    for (size_t i = 1; i < path.size() - 1; ++i) {
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();  // 窗口内点的坐标和
        size_t count = 0;  // 窗口内有效点的数量
        
        // 计算窗口内所有点的平均值
        for (int j = -static_cast<int>(window_size/2); j <= static_cast<int>(window_size/2); ++j) {
            size_t idx = i + j;  // 窗口内点的索引
            if (idx >= 0 && idx < path.size()) {  // 索引有效
                sum += path[idx];  // 累加坐标
                count++;  // 计数+1
            }
        }
        
        // 计算平均值并更新平滑路径
        if (count > 0) {
            smoothed[i] = sum / static_cast<double>(count);
        }
    }
    
    return smoothed;  // 返回平滑后的路径
}

} // namespace mid360_avoidance