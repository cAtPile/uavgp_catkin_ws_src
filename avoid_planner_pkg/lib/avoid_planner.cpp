#include "avoid_planner.h"
#include "local_planner/planner_functions.h"  // 复用路径点计算工具

namespace avoidance {

AvoidPlanner::AvoidPlanner(ros::NodeHandle& nh) : nh_(nh) {
    // 初始化订阅器
    goal_data_sub_ = nh.subscribe<apoc_msgs::apoc_setgoal_pose>(
        "apoc/setgoal/data", 10, &AvoidPlanner::goalDataCallback, this);
    goal_action_sub_ = nh.subscribe<std_msgs::Bool>(
        "apoc/setgoal/action", 10, &AvoidPlanner::goalActionCallback, this);

    // 初始化发布器（与原有规划器输出话题一致）
    setpoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
        "/local_planner/setpoint", 10);

    ROS_INFO("AvoidPlanner initialized");
}
    
void AvoidPlanner::goalDataCallback(const apoc_msgs::apoc_setgoal_pose::ConstPtr& msg) {
    // 接收目标点数据并存储
    current_goal_ = Eigen::Vector3f(
        static_cast<float>(msg->goal_x),
        static_cast<float>(msg->goal_y),
        static_cast<float>(msg->goal_z)
    );
    has_new_goal_ = true;
    ROS_INFO("Received new goal: (%.2f, %.2f, %.2f)", 
             current_goal_.x(), current_goal_.y(), current_goal_.z());
}

void AvoidPlanner::goalActionCallback(const std_msgs::Bool::ConstPtr& msg) {
    // 接收动作指令（true表示开始规划，false表示停止）
    start_planning_ = msg->data;
    if (start_planning_) {
        ROS_INFO("Start planning triggered");
        if (has_new_goal_) {
            executePlanner();  // 立即执行规划
        } else {
            ROS_WARN("No goal data received, cannot start planning");
        }
    } else {
        ROS_INFO("Planning stopped");
    }
}

void AvoidPlanner::executePlanner() {
    // 1. 设置目标点到本地规划器
    local_planner_.setGoal(current_goal_);

    // 2. 执行路径规划（复用原有逻辑）
    local_planner_.runPlanner();  // 触发规划计算

    // 3. 获取规划结果中的下一步定点
    avoidanceOutput planner_output = local_planner_.getAvoidanceOutput();
    std::vector<Eigen::Vector3f> path = planner_output.path_node_positions;

    // 4. 从路径中提取下一步定点（复用工具函数）
    Eigen::Vector3f next_setpoint;
    if (!path.empty() && getSetpointFromPath(
        path, 
        planner_output.last_path_time, 
        planner_output.cruise_velocity, 
        ros::Time::now(), 
        next_setpoint)) {

        // 5. 发布定点消息
        geometry_msgs::PoseStamped setpoint_msg;
        setpoint_msg.header.frame_id = "local_origin";  // 与原有坐标系保持一致
        setpoint_msg.header.stamp = ros::Time::now();
        setpoint_msg.pose.position.x = next_setpoint.x();
        setpoint_msg.pose.position.y = next_setpoint.y();
        setpoint_msg.pose.position.z = next_setpoint.z();
        setpoint_msg.pose.orientation.w = 1.0;  // 忽略姿态
        setpoint_pub_.publish(setpoint_msg);
    } else {
        ROS_WARN("Failed to generate valid setpoint");
    }
}

}  // namespace avoidance