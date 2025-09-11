#include "apoc_pkg/apoc.h"
#include <ros/ros.h>
#include <stdexcept>  // 用于抛出标准异常

// 从参数服务器读取配置
const std::string FLIGHT_TIMEOUT_PARAM = "flight_timeout";  // 飞行超时参数名
const std::string TAKEOFF_ALTITUDE_PARAM = "takeoff_altitude";  // 起飞高度参数名
const std::string REACH_TOL_DIST_PARAM = "reach_tolerance_distance";  // 距离到达阈值

// 默认参数
const float DEFAULT_FLIGHT_TIMEOUT = 30.0f;    // 默认飞行超时：30秒
const float DEFAULT_TAKEOFF_ALTITUDE = 1.0f;   // 默认起飞高度：1米
const float DEFAULT_REACH_TOL_DIST = 0.1f;     // 默认距离阈值：0.1米

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "apoc_takeoff_test_node", ros::init_options::AnonymousName);
    ROS_INFO("=== APOC Takeoff Test Node Started ===");

    // 创建ROS节点句柄
    ros::NodeHandle nh("~");  // 私有命名空间

    // 读取配置参数
    float flight_timeout, takeoff_altitude, reach_tol_dist;
    
    // 读取飞行超时参数
    if (!nh.getParam(FLIGHT_TIMEOUT_PARAM, flight_timeout)) {
        ROS_WARN("Param '%s' not found, use default: %.1fs", 
                 FLIGHT_TIMEOUT_PARAM.c_str(), DEFAULT_FLIGHT_TIMEOUT);
        flight_timeout = DEFAULT_FLIGHT_TIMEOUT;
    }
    
    // 读取起飞高度参数
    if (!nh.getParam(TAKEOFF_ALTITUDE_PARAM, takeoff_altitude)) {
        ROS_WARN("Param '%s' not found, use default: %.1fm", 
                 TAKEOFF_ALTITUDE_PARAM.c_str(), DEFAULT_TAKEOFF_ALTITUDE);
        takeoff_altitude = DEFAULT_TAKEOFF_ALTITUDE;
    }
    
    // 读取距离阈值参数
    if (!nh.getParam(REACH_TOL_DIST_PARAM, reach_tol_dist)) {
        ROS_WARN("Param '%s' not found, use default: %.2fm", 
                 REACH_TOL_DIST_PARAM.c_str(), DEFAULT_REACH_TOL_DIST);
        reach_tol_dist = DEFAULT_REACH_TOL_DIST;
    }

    ROS_INFO("Test Parameters - Takeoff Altitude: %.1fm, Timeout: %.1fs, Tolerance: %.2fm",
             takeoff_altitude, flight_timeout, reach_tol_dist);

    // 创建apoc类实例
    apoc apoc_control;

    // 初始化主循环频率
    ros::Rate main_rate(10);
    ros::Time test_start_time = ros::Time::now();

    try {
        // -------------------------- 步骤1：连接飞控 --------------------------
        ROS_INFO("\n[Step 1/3] Connecting to FCU (Timeout: %.1fs)...", flight_timeout);
        while (ros::ok() && !apoc_control.connectSwitch()) {
            if ((ros::Time::now() - test_start_time).toSec() > flight_timeout) {
                throw std::runtime_error(
                    "Connect to FCU timed out (exceed " + std::to_string(flight_timeout) + "s)"
                );
            }
            ros::spinOnce();
            main_rate.sleep();
        }
        ROS_INFO("[Step 1/3] Successfully connected to FCU!");

        // -------------------------- 步骤2：解锁无人机 --------------------------
        ROS_INFO("\n[Step 2/3] Arming vehicle (Timeout: %.1fs)...", flight_timeout);
        while (ros::ok() && !apoc_control.armSwitch(1)) {  // arm_key=1 → 解锁
            if ((ros::Time::now() - test_start_time).toSec() > flight_timeout) {
                throw std::runtime_error("Arm vehicle timed out");
            }
            ros::spinOnce();
            main_rate.sleep();
        }
        ROS_INFO("[Step 2/3] Vehicle armed successfully!");

        // -------------------------- 步骤3：切换到OFFBOARD模式 --------------------------
        ROS_INFO("\n[Step 3/4] Switching to OFFBOARD mode (Timeout: %.1fs)...", flight_timeout);
        while (ros::ok() && !apoc_control.modeSwitch(1)) {  // mode_key=1 → OFFBOARD
            if ((ros::Time::now() - test_start_time).toSec() > flight_timeout) {
                throw std::runtime_error("Switch to OFFBOARD mode timed out");
            }
            ros::spinOnce();
            main_rate.sleep();
        }
        ROS_INFO("[Step 3/4] Successfully switched to OFFBOARD mode!");

        // -------------------------- 步骤4：执行起飞 --------------------------
        ROS_INFO("\n[Step 4/4] Executing takeoff to %.1fm (Timeout: %.1fs)...", 
                 takeoff_altitude, flight_timeout);
        
        // 重置超时计时器，专门用于起飞过程
        ros::Time takeoff_start_time = ros::Time::now();
        bool takeoff_success = false;
        
        // 循环检查起飞是否成功
        while (ros::ok()) {
            takeoff_success = apoc_control.takeoffSwitch(takeoff_altitude);
            
            // 检查起飞是否成功
            if (takeoff_success) {
                ROS_INFO("[Step 4/4] Takeoff completed successfully! Reached target altitude.");
                break;
            }
            
            // 检查起飞是否超时
            if ((ros::Time::now() - takeoff_start_time).toSec() > flight_timeout) {
                throw std::runtime_error("Takeoff timed out");
            }
            
            ros::spinOnce();
            main_rate.sleep();
        }

    } catch (const std::exception& e) {
        // 异常处理：打印错误并紧急上锁
        ROS_ERROR("\nTest aborted due to error: %s", e.what());
        apoc_control.armSwitch(0);  // 紧急上锁
        ros::shutdown();
        return -1;
    }

    // 测试完成
    ROS_INFO("\n=== All Takeoff Test Steps Completed Successfully! ===");

    // （可选）测试后悬停一段时间再降落或上锁
    ROS_INFO("Hovering for 5 seconds...");
    ros::Duration(5.0).sleep();
    
    // （可选）自动上锁
    apoc_control.armSwitch(0);
    ROS_INFO("Vehicle disarmed after test completion.");

    ros::shutdown();
    return 0;
}
    
