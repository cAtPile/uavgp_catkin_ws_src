#include "avoid_planner_pkg/mavros_interface.h"
#include "avoid_planner_pkg/utils.h"
#include <tf2/LinearMath/Quaternion.h>
#include <mavros_msgs/CommandBool.h>
#include <std_msgs/Float64.h>
#include <ros/console.h>
#include <cmath>

namespace mid360_avoidance {

MavrosInterface::MavrosInterface(ros::NodeHandle& nh) : nh_(nh) {
    // 获取MAVROS命名空间（默认为"mavros"）
    nh_.param<std::string>("mavros_namespace", mavros_namespace_, "mavros");
    nh_.param<double>("offboard_rate", offboard_rate_, 30.0);
    
    // 订阅MAVROS话题
    state_sub_ = nh_.subscribe(mavros_namespace_ + "/state", 10,
                              &MavrosInterface::stateCallback, this);
    pose_sub_ = nh_.subscribe(mavros_namespace_ + "/local_position/pose", 10,
                             &MavrosInterface::poseCallback, this);
    velocity_sub_ = nh_.subscribe(mavros_namespace_ + "/local_position/velocity_local", 10,
                                 &MavrosInterface::velocityCallback, this);
    
    // 初始化发布者
    target_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        mavros_namespace_ + "/setpoint_position/local", 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>(
        mavros_namespace_ + "/setpoint_trajectory/local", 10);
    
    // 初始化服务客户端
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
        mavros_namespace_ + "/set_mode");
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
        mavros_namespace_ + "/cmd/arming");
    
    ROS_INFO("MavrosInterface initialized with namespace: %s", mavros_namespace_.c_str());
}

VehicleState MavrosInterface::getVehicleState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return vehicle_state_;
}

Eigen::Vector3d MavrosInterface::getCurrentPosition() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return vehicle_state_.position;
}

Eigen::Vector3d MavrosInterface::getCurrentVelocity() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return vehicle_state_.velocity;
}

double MavrosInterface::getCurrentYaw() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return vehicle_state_.yaw;
}

void MavrosInterface::sendPath(const std::vector<Eigen::Vector3d>& path, 
                              const std::string& frame_id) {
    if (path.empty()) {
        ROS_WARN_THROTTLE(1.0, "Trying to send empty path");
        return;
    }
    
    // 转换路径为nav_msgs::Path消息
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = frame_id;
    
    for (const auto& waypoint : path) {
        path_msg.poses.push_back(eigenToPoseStamped(waypoint, frame_id));
    }
    
    // 发布路径
    path_pub_.publish(path_msg);
    
    // 同时发送路径第一个点作为当前目标点（满足OFFBOARD模式最低更新频率要求）
    sendTargetPosition(path.front(), std::nan(""), frame_id);
}

void MavrosInterface::sendTargetPosition(const Eigen::Vector3d& target, 
                                        double yaw,
                                        const std::string& frame_id) {
    geometry_msgs::PoseStamped target_pose = eigenToPoseStamped(target, frame_id, yaw);
    target_pos_pub_.publish(target_pose);
}

bool MavrosInterface::setFlightMode(const std::string& mode) {
    if (!isConnected()) {
        ROS_WARN("Cannot set mode - not connected to vehicle");
        return false;
    }
    
    mavros_msgs::SetMode set_mode_srv;
    set_mode_srv.request.base_mode = 0;
    set_mode_srv.request.custom_mode = mode;
    
    if (set_mode_client_.call(set_mode_srv) && set_mode_srv.response.mode_sent) {
        ROS_INFO("Flight mode set to: %s", mode.c_str());
        return true;
    } else {
        ROS_WARN("Failed to set flight mode to: %s", mode.c_str());
        return false;
    }
}

bool MavrosInterface::armVehicle() {
    if (!isConnected()) {
        ROS_WARN("Cannot arm - not connected to vehicle");
        return false;
    }
    
    if (isArmed()) {
        ROS_INFO("Vehicle already armed");
        return true;
    }
    
    mavros_msgs::CommandBool arm_srv;
    arm_srv.request.value = true;
    
    if (arming_client_.call(arm_srv) && arm_srv.response.success) {
        ROS_INFO("Vehicle armed successfully");
        return true;
    } else {
        ROS_WARN("Failed to arm vehicle");
        return false;
    }
}

bool MavrosInterface::disarmVehicle() {
    if (!isConnected()) {
        ROS_WARN("Cannot disarm - not connected to vehicle");
        return false;
    }
    
    if (!isArmed()) {
        ROS_INFO("Vehicle already disarmed");
        return true;
    }
    
    mavros_msgs::CommandBool arm_srv;
    arm_srv.request.value = false;
    
    if (arming_client_.call(arm_srv) && arm_srv.response.success) {
        ROS_INFO("Vehicle disarmed successfully");
        return true;
    } else {
        ROS_WARN("Failed to disarm vehicle");
        return false;
    }
}

bool MavrosInterface::isOffboardMode() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return vehicle_state_.mode == "OFFBOARD";
}

bool MavrosInterface::isArmed() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return vehicle_state_.armed;
}

bool MavrosInterface::isConnected() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return vehicle_state_.connected;
}

void MavrosInterface::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    vehicle_state_.state = *msg;
    vehicle_state_.connected = msg->connected;
    vehicle_state_.armed = msg->armed;
    vehicle_state_.mode = msg->mode;
}

void MavrosInterface::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    vehicle_state_.pose = *msg;
    
    // 更新位置向量
    vehicle_state_.position.x() = msg->pose.position.x;
    vehicle_state_.position.y() = msg->pose.position.y;
    vehicle_state_.position.z() = msg->pose.position.z;
    
    // 从四元数提取偏航角
    Eigen::Vector3d rpy = quaternionToYaw(msg->pose.orientation);
    vehicle_state_.yaw = rpy.z();
}

void MavrosInterface::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    vehicle_state_.twist = *msg;
    
    // 更新速度向量
    vehicle_state_.velocity.x() = msg->twist.linear.x;
    vehicle_state_.velocity.y() = msg->twist.linear.y;
    vehicle_state_.velocity.z() = msg->twist.linear.z;
}

geometry_msgs::PoseStamped MavrosInterface::eigenToPoseStamped(const Eigen::Vector3d& vector,
                                                              const std::string& frame_id,
                                                              double yaw) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id;
    
    // 设置位置
    pose.pose.position.x = vector.x();
    pose.pose.position.y = vector.y();
    pose.pose.position.z = vector.z();
    
    // 设置姿态（默认使用当前偏航角）
    if (std::isnan(yaw)) {
        // 使用当前偏航角
        std::lock_guard<std::mutex> lock(state_mutex_);
        yaw = vehicle_state_.yaw;
    }
    
    // 将偏航角转换为四元数
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    
    return pose;
}

} // namespace mid360_avoidance
