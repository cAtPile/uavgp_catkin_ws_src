#include "avoid_planner_pkg/mavros_interface.h"
#include "avoid_planner_pkg/utils.h"
#include <tf2/LinearMath/Quaternion.h>
#include <mavros_msgs/CommandBool.h>
#include <std_msgs/Float64.h>
#include <ros/console.h>
#include <cmath>

namespace mid360_avoidance {

/**
 * @brief MavrosInterface类构造函数（初始化MAVROS接口）
 * @details 初始化ROS节点句柄、参数读取、话题订阅/发布以及服务客户端，
 * 是连接避障规划器与MAVROS（无人机控制中间件）的核心接口
 * @param nh ROS节点句柄（用于创建订阅者、发布者和服务客户端）
 */
MavrosInterface::MavrosInterface(ros::NodeHandle& nh) : nh_(nh) {
    // 从参数服务器读取MAVROS命名空间（默认值为"mavros"，支持多无人机场景）
    nh_.param<std::string>("mavros_namespace", mavros_namespace_, "mavros");
    // 读取OFFBOARD模式下的控制指令发布频率（默认30Hz，满足PX4最低更新要求）
    nh_.param<double>("offboard_rate", offboard_rate_, 30.0);
    
    // 订阅MAVROS核心状态话题（无人机连接状态、武装状态、飞行模式等）
    state_sub_ = nh_.subscribe(mavros_namespace_ + "/state", 10,
                              &MavrosInterface::stateCallback, this);
    // 订阅无人机本地位置话题（ENU坐标系下的位置和姿态）
    pose_sub_ = nh_.subscribe(mavros_namespace_ + "/local_position/pose", 10,
                             &MavrosInterface::poseCallback, this);
    // 订阅无人机本地速度话题（ENU坐标系下的线速度）
    velocity_sub_ = nh_.subscribe(mavros_namespace_ + "/local_position/velocity_local", 10,
                                 &MavrosInterface::velocityCallback, this);
    
    // 初始化目标位置发布者（用于向MAVROS发送本地位置控制指令）
    target_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        mavros_namespace_ + "/setpoint_position/local", 10);
    // 初始化路径发布者（用于向MAVROS发送完整轨迹点列表）
    path_pub_ = nh_.advertise<nav_msgs::Path>(
        mavros_namespace_ + "/setpoint_trajectory/local", 10);
    
    // 初始化飞行模式设置服务客户端（用于切换OFFBOARD、STABILIZED等模式）
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
        mavros_namespace_ + "/set_mode");
    // 初始化无人机武装/解锁服务客户端（用于启动或关闭电机）
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
        mavros_namespace_ + "/cmd/arming");
    
    // 输出接口初始化完成信息（包含使用的MAVROS命名空间）
    ROS_INFO("MavrosInterface initialized with namespace: %s", mavros_namespace_.c_str());
}

/**
 * @brief 获取无人机当前完整状态
 * @details 线程安全地返回无人机的连接状态、武装状态、飞行模式、位置、速度和偏航角等信息
 * @return VehicleState 包含无人机所有状态的结构体
 */
VehicleState MavrosInterface::getVehicleState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);  // 加锁确保线程安全（防止状态读写冲突）
    return vehicle_state_;
}

/**
 * @brief 获取无人机当前位置（ENU坐标系）
 * @details 线程安全地返回无人机当前的三维位置向量（x/y/z轴，单位：米）
 * @return Eigen::Vector3d 位置向量（x:东向，y:北向，z:天向）
 */
Eigen::Vector3d MavrosInterface::getCurrentPosition() const {
    std::lock_guard<std::mutex> lock(state_mutex_);  // 线程安全锁
    return vehicle_state_.position;
}

/**
 * @brief 获取无人机当前速度（ENU坐标系）
 * @details 线程安全地返回无人机当前的三维线速度向量（x/y/z轴，单位：米/秒）
 * @return Eigen::Vector3d 速度向量（x:东向，y:北向，z:天向）
 */
Eigen::Vector3d MavrosInterface::getCurrentVelocity() const {
    std::lock_guard<std::mutex> lock(state_mutex_);  // 线程安全锁
    return vehicle_state_.velocity;
}

/**
 * @brief 获取无人机当前偏航角
 * @details 线程安全地返回无人机当前的偏航角（绕z轴旋转角度，单位：弧度，范围[-π, π]）
 * @return double 偏航角（0表示朝向东，逆时针为正）
 */
double MavrosInterface::getCurrentYaw() const {
    std::lock_guard<std::mutex> lock(state_mutex_);  // 线程安全锁
    return vehicle_state_.yaw;
}

/**
 * @brief 发送完整路径给无人机
 * @details 将规划器生成的路径点列表转换为MAVROS支持的nav_msgs::Path消息并发布，
 * 同时发送路径第一个点作为当前目标位置（满足OFFBOARD模式最低更新频率要求）
 * @param path 路径点列表（Eigen::Vector3d类型，ENU坐标系下的位置）
 * @param frame_id 坐标系ID（通常为"map"或"local_origin"，需与MAVROS配置一致）
 */
void MavrosInterface::sendPath(const std::vector<Eigen::Vector3d>& path, 
                              const std::string& frame_id) {
    if (path.empty()) {  // 检查路径是否为空
        ROS_WARN_THROTTLE(1.0, "Trying to send empty path");  // 1秒内仅警告一次（避免刷屏）
        return;
    }
    
    // 构建nav_msgs::Path消息（包含路径所有点的时间戳、坐标系和姿态信息）
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();  // 设置消息时间戳
    path_msg.header.frame_id = frame_id;       // 设置坐标系ID
    
    // 将每个路径点转换为geometry_msgs::PoseStamped并添加到路径消息中
    for (const auto& waypoint : path) {
        path_msg.poses.push_back(eigenToPoseStamped(waypoint, frame_id));
    }
    
    // 发布路径消息（供MAVROS或地面站显示完整轨迹）
    path_pub_.publish(path_msg);
    
    // 发送路径第一个点作为当前目标位置（OFFBOARD模式要求至少2Hz的控制指令更新）
    sendTargetPosition(path.front(), std::nan(""), frame_id);
}

/**
 * @brief 发送单个目标位置给无人机
 * @details 将目标位置转换为MAVROS支持的geometry_msgs::PoseStamped消息并发布，
 * 用于OFFBOARD模式下的位置控制
 * @param target 目标位置（Eigen::Vector3d类型，ENU坐标系）
 * @param yaw 目标偏航角（单位：弧度，若为NaN则使用当前偏航角）
 * @param frame_id 坐标系ID（与路径发布的坐标系一致）
 */
void MavrosInterface::sendTargetPosition(const Eigen::Vector3d& target, 
                                        double yaw,
                                        const std::string& frame_id) {
    // 将Eigen向量转换为带姿态的PoseStamped消息（处理偏航角）
    geometry_msgs::PoseStamped target_pose = eigenToPoseStamped(target, frame_id, yaw);
    // 发布目标位置消息（MAVROS接收后控制无人机飞向该位置）
    target_pos_pub_.publish(target_pose);
}

/**
 * @brief 设置无人机飞行模式
 * @details 通过MAVROS的set_mode服务切换无人机飞行模式（如OFFBOARD、STABILIZED等）
 * @param mode 目标飞行模式字符串（需与PX4支持的模式一致，如"OFFBOARD"、"AUTO.MISSION"）
 * @return bool 模式设置成功返回true，失败返回false
 */
bool MavrosInterface::setFlightMode(const std::string& mode) {
    if (!isConnected()) {  // 检查是否与无人机建立连接
        ROS_WARN("Cannot set mode - not connected to vehicle");
        return false;
    }
    
    // 构建飞行模式设置服务请求
    mavros_msgs::SetMode set_mode_srv;
    set_mode_srv.request.base_mode = 0;                // 基础模式（0表示不使用）
    set_mode_srv.request.custom_mode = mode;           // 自定义模式（目标飞行模式）
    
    // 调用服务并检查结果
    if (set_mode_client_.call(set_mode_srv) && set_mode_srv.response.mode_sent) {
        ROS_INFO("Flight mode set to: %s", mode.c_str());  // 模式设置成功
        return true;
    } else {
        ROS_WARN("Failed to set flight mode to: %s", mode.c_str());  // 模式设置失败
        return false;
    }
}

/**
 * @brief 武装无人机（启动电机）
 * @details 通过MAVROS的cmd/arming服务请求武装无人机，仅在连接状态下有效
 * @return bool 武装成功返回true，失败或已武装返回false
 */
bool MavrosInterface::armVehicle() {
    if (!isConnected()) {  // 检查是否与无人机建立连接
        ROS_WARN("Cannot arm - not connected to vehicle");
        return false;
    }
    
    if (isArmed()) {  // 检查是否已处于武装状态
        ROS_INFO("Vehicle already armed");
        return true;
    }
    
    // 构建武装服务请求（value=true表示武装）
    mavros_msgs::CommandBool arm_srv;
    arm_srv.request.value = true;
    
    // 调用服务并检查结果
    if (arming_client_.call(arm_srv) && arm_srv.response.success) {
        ROS_INFO("Vehicle armed successfully");  // 武装成功
        return true;
    } else {
        ROS_WARN("Failed to arm vehicle");  // 武装失败
        return false;
    }
}

/**
 * @brief 解除无人机武装（关闭电机）
 * @details 通过MAVROS的cmd/arming服务请求解除武装，仅在连接状态下有效
 * @return bool 解除武装成功返回true，失败或已解除武装返回false
 */
bool MavrosInterface::disarmVehicle() {
    if (!isConnected()) {  // 检查是否与无人机建立连接
        ROS_WARN("Cannot disarm - not connected to vehicle");
        return false;
    }
    
    if (!isArmed()) {  // 检查是否已处于解除武装状态
        ROS_INFO("Vehicle already disarmed");
        return true;
    }
    
    // 构建解除武装服务请求（value=false表示解除武装）
    mavros_msgs::CommandBool arm_srv;
    arm_srv.request.value = false;
    
    // 调用服务并检查结果
    if (arming_client_.call(arm_srv) && arm_srv.response.success) {
        ROS_INFO("Vehicle disarmed successfully");  // 解除武装成功
        return true;
    } else {
        ROS_WARN("Failed to disarm vehicle");  // 解除武装失败
        return false;
    }
}

/**
 * @brief 检查无人机是否处于OFFBOARD模式
 * @details 线程安全地判断当前飞行模式是否为"OFFBOARD"（外部控制模式）
 * @return bool 是OFFBOARD模式返回true，否则返回false
 */
bool MavrosInterface::isOffboardMode() const {
    std::lock_guard<std::mutex> lock(state_mutex_);  // 线程安全锁
    return vehicle_state_.mode == "OFFBOARD";
}

/**
 * @brief 检查无人机是否已武装
 * @details 线程安全地判断无人机电机是否处于启动状态（武装状态）
 * @return bool 已武装返回true，未武装返回false
 */
bool MavrosInterface::isArmed() const {
    std::lock_guard<std::mutex> lock(state_mutex_);  // 线程安全锁
    return vehicle_state_.armed;
}

/**
 * @brief 检查无人机是否与MAVROS建立连接
 * @details 线程安全地判断无人机是否通过MAVLink与MAVROS建立通信
 * @return bool 已连接返回true，未连接返回false
 */
bool MavrosInterface::isConnected() const {
    std::lock_guard<std::mutex> lock(state_mutex_);  // 线程安全锁
    return vehicle_state_.connected;
}

/**
 * @brief MAVROS状态话题回调函数
 * @details 接收MAVROS发布的无人机状态消息，更新本地存储的状态信息（连接、武装、模式）
 * @param msg 指向mavros_msgs::State消息的常量指针
 */
void MavrosInterface::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);  // 线程安全锁（防止多回调冲突）
    vehicle_state_.state = *msg;          // 保存完整状态消息
    vehicle_state_.connected = msg->connected;  // 更新连接状态
    vehicle_state_.armed = msg->armed;          // 更新武装状态
    vehicle_state_.mode = msg->mode;            // 更新飞行模式
}

/**
 * @brief MAVROS本地位置话题回调函数
 * @details 接收MAVROS发布的无人机本地位置消息，更新位置向量和偏航角
 * @param msg 指向geometry_msgs::PoseStamped消息的常量指针
 */
void MavrosInterface::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);  // 线程安全锁
    vehicle_state_.pose = *msg;  // 保存完整姿态消息
    
    // 更新位置向量（从Pose消息中提取x/y/z坐标）
    vehicle_state_.position.x() = msg->pose.position.x;
    vehicle_state_.position.y() = msg->pose.position.y;
    vehicle_state_.position.z() = msg->pose.position.z;
    
    // 从四元数姿态中提取偏航角（绕z轴旋转角度）
    Eigen::Vector3d rpy = quaternionToYaw(msg->pose.orientation);
    vehicle_state_.yaw = rpy.z();  // 保存偏航角（rpy.z()对应偏航）
}

/**
 * @brief MAVROS本地速度话题回调函数
 * @details 接收MAVROS发布的无人机本地速度消息，更新速度向量
 * @param msg 指向geometry_msgs::TwistStamped消息的常量指针
 */
void MavrosInterface::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);  // 线程安全锁
    vehicle_state_.twist = *msg;  // 保存完整速度消息
    
    // 更新速度向量（从Twist消息中提取线性速度x/y/z分量）
    vehicle_state_.velocity.x() = msg->twist.linear.x;
    vehicle_state_.velocity.y() = msg->twist.linear.y;
    vehicle_state_.velocity.z() = msg->twist.linear.z;
}

/**
 * @brief 将Eigen位置向量转换为ROS PoseStamped消息
 * @details 生成包含位置和姿态的PoseStamped消息，支持自定义偏航角（默认使用当前偏航）
 * @param vector Eigen位置向量（x/y/z，ENU坐标系）
 * @param frame_id 消息的坐标系ID
 * @param yaw 目标偏航角（单位：弧度，若为NaN则使用当前无人机偏航角）
 * @return geometry_msgs::PoseStamped 转换后的PoseStamped消息
 */
geometry_msgs::PoseStamped MavrosInterface::eigenToPoseStamped(const Eigen::Vector3d& vector,
                                                              const std::string& frame_id,
                                                              double yaw) {
    geometry_msgs::PoseStamped pose;  // 初始化PoseStamped消息
    pose.header.stamp = ros::Time::now();  // 设置消息时间戳
    pose.header.frame_id = frame_id;       // 设置坐标系ID
    
    // 设置位置信息（从Eigen向量提取x/y/z）
    pose.pose.position.x = vector.x();
    pose.pose.position.y = vector.y();
    pose.pose.position.z = vector.z();
    
    // 处理姿态：若偏航角为NaN，使用当前无人机偏航角
    if (std::isnan(yaw)) {
        std::lock_guard<std::mutex> lock(state_mutex_);  // 线程安全锁
        yaw = vehicle_state_.yaw;  // 读取当前偏航角
    }
    
    // 将偏航角转换为四元数（滚转和俯仰设为0，仅控制偏航）
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);  // 滚转(roll)=0，俯仰(pitch)=0，偏航(yaw)=输入值
    // 将四元数赋值到Pose消息
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    
    return pose;  // 返回转换后的PoseStamped消息
}

} // namespace mid36