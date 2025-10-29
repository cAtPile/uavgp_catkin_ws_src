/**
 * @file goalCB.cpp
 * @brief 目标回调
 * @details
 * @author apoc
 * @date 2025/10/18
 */
#include"avoid_planner_pkg/avoid_planner.h"

/**
 * @brief gaolCB.cpp
 * @details 获得目标位置
 *          获得当前位置
 *          计算相对极坐标位置
 *          生成势场图
 *          计算目标方向
 * @see generatePFpotmap();
 */
#include"avoid_planner_pkg/avoid_planner.h"

/**
 * @brief 处理目标回调函数
 * @details 接收目标位置和当前位置，计算相对位置并生成势场，最终确定运动方向
 */
void AvoidPlanner::goalCB(){
    // 接收新目标
    const auto& goal = *as_.acceptNewGoal();
    
    // 存储当前目标
    current_goal_ = goal;

    // 提取当前位置（从Action Goal消息中）
    Eigen::Vector3d current_position;
    current_position.x() = goal.current_pose_x;  
    current_position.y() = goal.current_pose_y;  
    current_position.z() = goal.current_pose_z;  

    // 提取目标位置
    Eigen::Vector3d goal_position;
    goal_position.x() = goal.goal_x;
    goal_position.y() = goal.goal_y;
    goal_position.z() = goal.goal_z;

    // 计算相对位置
    Eigen::Vector3d relative_pos = goal_position - current_position;
    
    // 计算极坐标参数
    double distance = relative_pos.norm();
    double azimuth = atan2(relative_pos.y(), relative_pos.x());  // 方位角（绕z轴）
    double elevation = atan2(relative_pos.z(), relative_pos.head<2>().norm());  // 仰角（绕y轴）

    // 生成势场图
    generatePFpotmap(azimuth, elevation, distance);

    // 计算最终运动方向
    std::lock_guard<std::mutex> lock(data_mutex_);  // 保护共享数据
    current_direction_ = current_polar_field_.force_vector.normalized();

    // 处理命令
    if (goal.cmd == 1) {  // 1为启动命令
        is_running_ = true;
    } else if (goal.cmd == 0) {  // 0为停止命令
        is_running_ = false;
    }

    // 发布反馈
    feedback_.current_direction.x = current_direction_.x();
    feedback_.current_direction.y = current_direction_.y();
    feedback_.current_direction.z = current_direction_.z();
    as_.publishFeedback(feedback_);

    // 如果已经到达目标（距离小于阈值），返回结果
    if (distance < 0.5) {  // 假设0.5m为到达阈值
        // 利用Vector3类型的current_direction，直接赋值其x/y/z子字段
        result_.current_direction.x = 0.0;
        result_.current_direction.y = 0.0;
        result_.current_direction.z = 0.0;
        as_.setSucceeded(result_);
    }
}