//MAVROS通信接口声明
#ifndef AVOID_PLANNER_MAVROS_INTERFACE_H
#define AVOID_PLANNER_MAVROS_INTERFACE_H

#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include <mutex>
#include <string>

namespace avoid_planner {

/**
 * @struct VehicleState
 * @brief 无人机状态结构体，包含位置、速度、姿态等关键信息
 */
struct VehicleState {
    geometry_msgs::PoseStamped pose;          // 位置和姿态(ENU坐标系)
    geometry_msgs::TwistStamped twist;        // 线速度和角速度(ENU坐标系)
    mavros_msgs::State state;                 // MAVROS状态信息
    Eigen::Vector3d position;                 // 位置向量(ENU坐标系)
    Eigen::Vector3d velocity;                 // 速度向量(ENU坐标系)
    double yaw;                               // 偏航角(弧度)
    bool connected;                           // 连接状态
    bool armed;                               // 解锁状态
    std::string mode;                         // 飞行模式
    
    // 构造函数
    VehicleState() : yaw(0.0), connected(false), armed(false), mode("") {}
};

/**
 * @class MavrosInterface
 * @brief MAVROS通信接口类，负责与PX4飞控进行数据交互
 */
class MavrosInterface {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    explicit MavrosInterface(ros::NodeHandle& nh);
    
    /**
     * @brief 析构函数
     */
    ~MavrosInterface() = default;
    
    /**
     * @brief 获取无人机当前状态
     * @return 无人机状态结构体
     */
    VehicleState getVehicleState() const;
    
    /**
     * @brief 获取无人机当前位置(ENU坐标系)
     * @return 位置向量
     */
    Eigen::Vector3d getCurrentPosition() const;
    
    /**
     * @brief 获取无人机当前速度(ENU坐标系)
     * @return 速度向量
     */
    Eigen::Vector3d getCurrentVelocity() const;
    
    /**
     * @brief 获取无人机当前偏航角
     * @return 偏航角(弧度)
     */
    double getCurrentYaw() const;
    
    /**
     * @brief 发送局部路径到PX4
     * @param path 路径航点序列(ENU坐标系)
     * @param frame_id 坐标系ID，默认为"map"
     */
    void sendPath(const std::vector<Eigen::Vector3d>& path, 
                 const std::string& frame_id = "map");
    
    /**
     * @brief 发送单个目标点到PX4
     * @param target 目标点位置(ENU坐标系)
     * @param yaw 目标偏航角(弧度)，默认为NaN(使用当前偏航)
     * @param frame_id 坐标系ID，默认为"map"
     */
    void sendTargetPosition(const Eigen::Vector3d& target, 
                           double yaw = std::nan(""),
                           const std::string& frame_id = "map");
    
    /**
     * @brief 设置无人机飞行模式
     * @param mode 模式名称，如"OFFBOARD"、"AUTO.MISSION"等
     * @return 设置成功返回true
     */
    bool setFlightMode(const std::string& mode);
    
    /**
     * @brief 解锁无人机
     * @return 解锁成功返回true
     */
    bool armVehicle();
    
    /**
     * @brief 锁定无人机
     * @return 锁定成功返回true
     */
    bool disarmVehicle();
    
    /**
     * @brief 检查是否处于OFFBOARD模式
     * @return 是返回true
     */
    bool isOffboardMode() const;
    
    /**
     * @brief 检查无人机是否已解锁
     * @return 已解锁返回true
     */
    bool isArmed() const;
    
    /**
     * @brief 检查与飞控的连接状态
     * @return 连接正常返回true
     */
    bool isConnected() const;

private:
    /**
     * @brief MAVROS状态回调函数
     * @param msg MAVROS状态消息
     */
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    
    /**
     * @brief 位置姿态回调函数
     * @param msg 位置姿态消息
     */
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    
    /**
     * @brief 速度回调函数
     * @param msg 速度消息
     */
    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    
    /**
     * @brief 转换Eigen向量到PoseStamped消息
     * @param vector Eigen位置向量
     * @param frame_id 坐标系ID
     * @param yaw 偏航角(弧度)，默认为NaN
     * @return 转换后的PoseStamped消息
     */
    geometry_msgs::PoseStamped eigenToPoseStamped(const Eigen::Vector3d& vector,
                                                 const std::string& frame_id,
                                                 double yaw = std::nan(""));

    // ROS相关成员
    ros::NodeHandle nh_;                          // ROS节点句柄
    ros::Subscriber state_sub_;                   // 状态订阅者
    ros::Subscriber pose_sub_;                    // 位置姿态订阅者
    ros::Subscriber velocity_sub_;                // 速度订阅者
    ros::Publisher target_pos_pub_;               // 目标位置发布者
    ros::Publisher path_pub_;                     // 路径发布者
    ros::ServiceClient set_mode_client_;          // 模式设置服务客户端
    ros::ServiceClient arming_client_;            // 解锁服务客户端
    
    // 无人机状态
    VehicleState vehicle_state_;                  // 无人机状态
    mutable std::mutex state_mutex_;              // 状态数据锁
    
    // 配置参数
    std::string mavros_namespace_;                // MAVROS命名空间
    double offboard_rate_;                        // OFFBOARD模式发布频率(Hz)
};

} // namespace avoid_planner

#endif // AVOID_PLANNER_MAVROS_INTERFACE_H
