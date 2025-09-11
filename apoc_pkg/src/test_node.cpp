#include "apoc_pkg/apoc.h"
#include <ros/ros.h>
#include <stdexcept>  // 用于抛出标准异常

// 从参数服务器读取配置
const std::string FLIGHT_TIMEOUT_PARAM = "flight_timeout";      // 飞行超时参数名
const std::string TAKEOFF_ALTITUDE_PARAM = "takeoff_altitude";  // 起飞高度参数名
const std::string HOVER_TIME_PARAM = "hover_time";              // 悬停时间参数名
const std::string REACH_TOL_DIST_PARAM = "reach_tolerance_distance";  // 距离到达阈值
const std::string LANDING_TIMEOUT_PARAM = "landing_timeout";    // 降落超时参数名

// 默认参数
const float DEFAULT_FLIGHT_TIMEOUT = 30.0f;    // 默认飞行超时：30秒
const float DEFAULT_TAKEOFF_ALTITUDE = 1.0f;   // 默认起飞高度：1米
const float DEFAULT_HOVER_TIME = 2.0f;        // 默认悬停时间：10秒
const float DEFAULT_REACH_TOL_DIST = 0.1f;     // 默认距离阈值：0.1米
const float DEFAULT_LANDING_TIMEOUT = 60.0f;   // 默认降落超时：60秒

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "apoc_complete_test_node", ros::init_options::AnonymousName);
    ROS_INFO("=== APOC Complete Flight Test Node Started ===");

    // 创建ROS节点句柄
    ros::NodeHandle nh("~");  // 私有命名空间

    // 读取配置参数
    float flight_timeout, takeoff_altitude, hover_time, reach_tol_dist, landing_timeout;
    
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
    
    // 读取悬停时间参数
    if (!nh.getParam(HOVER_TIME_PARAM, hover_time)) {
        ROS_WARN("Param '%s' not found, use default: %.1fs", 
                 HOVER_TIME_PARAM.c_str(), DEFAULT_HOVER_TIME);
        hover_time = DEFAULT_HOVER_TIME;
    }
    
    // 读取距离阈值参数
    if (!nh.getParam(REACH_TOL_DIST_PARAM, reach_tol_dist)) {
        ROS_WARN("Param '%s' not found, use default: %.2fm", 
                 REACH_TOL_DIST_PARAM.c_str(), DEFAULT_REACH_TOL_DIST);
        reach_tol_dist = DEFAULT_REACH_TOL_DIST;
    }
    
    // 读取降落超时参数
    if (!nh.getParam(LANDING_TIMEOUT_PARAM, landing_timeout)) {
        ROS_WARN("Param '%s' not found, use default: %.1fs", 
                 LANDING_TIMEOUT_PARAM.c_str(), DEFAULT_LANDING_TIMEOUT);
        landing_timeout = DEFAULT_LANDING_TIMEOUT;
    }

    ROS_INFO("Test Parameters - Takeoff Altitude: %.1fm, Hover Time: %.1fs, Timeout: %.1fs, Landing Timeout: %.1fs",
             takeoff_altitude, hover_time, flight_timeout, landing_timeout);

    // 创建apoc类实例
    apoc apoc_control;

    // 初始化主循环频率
    ros::Rate main_rate(10);
    ros::Time test_start_time = ros::Time::now();

    try {
        // -------------------------- 步骤1：连接飞控 --------------------------
        ROS_INFO("\n[Step 1/7] Connecting to FCU (Timeout: %.1fs)...", flight_timeout);
        while (ros::ok() && !apoc_control.connectSwitch()) {
            if ((ros::Time::now() - test_start_time).toSec() > flight_timeout) {
                throw std::runtime_error(
                    "Connect to FCU timed out (exceed " + std::to_string(flight_timeout) + "s)"
                );
            }
            ros::spinOnce();
            main_rate.sleep();
        }
        ROS_INFO("[Step 1/7] Successfully connected to FCU!");

        // -------------------------- 步骤2：解锁无人机 --------------------------
        ROS_INFO("\n[Step 2/7] Arming vehicle (Timeout: %.1fs)...", flight_timeout);
        while (ros::ok() && !apoc_control.armSwitch(1)) {  // arm_key=1 → 解锁
            if ((ros::Time::now() - test_start_time).toSec() > flight_timeout) {
                throw std::runtime_error("Arm vehicle timed out");
            }
            ros::spinOnce();
            main_rate.sleep();
        }
        ROS_INFO("[Step 2/7] Vehicle armed successfully!");

        // -------------------------- 步骤3：切换到OFFBOARD模式 --------------------------
        ROS_INFO("\n[Step 3/7] Switching to OFFBOARD mode (Timeout: %.1fs)...", flight_timeout);
        while (ros::ok() && !apoc_control.modeSwitch(1)) {  // mode_key=1 → OFFBOARD
            if ((ros::Time::now() - test_start_time).toSec() > flight_timeout) {
                throw std::runtime_error("Switch to OFFBOARD mode timed out");
            }
            ros::spinOnce();
            main_rate.sleep();
        }
        ROS_INFO("[Step 3/7] Successfully switched to OFFBOARD mode!");

        // -------------------------- 步骤4：执行起飞到(0,0,1,0) --------------------------
        ROS_INFO("\n[Step 4/7] Executing takeoff to (0,0,1,0) (Timeout: %.1fs)...", 
                 flight_timeout);
        
        // 重置超时计时器，专门用于起飞过程
        ros::Time takeoff_start_time = ros::Time::now();
        bool takeoff_success = false;
        
        // 循环检查起飞是否成功
        while (ros::ok()) {
            takeoff_success = apoc_control.takeoffSwitch(takeoff_altitude);
            
            // 检查起飞是否成功
            if (takeoff_success) {
                ROS_INFO("[Step 4/7] Takeoff completed successfully! Reached target altitude.");
                break;
            }
            
            // 检查起飞是否超时
            if ((ros::Time::now() - takeoff_start_time).toSec() > flight_timeout) {
                throw std::runtime_error("Takeoff timed out");
            }
            
            ros::spinOnce();
            main_rate.sleep();
        }

        // -------------------------- 步骤5：悬停一段时间 --------------------------
        ROS_INFO("\n[Step 5/7] Starting hover for %.1f seconds before relative flight...", hover_time);
        
        // 记录悬停开始时的位置和时间
        ros::Time hover_start_time = ros::Time::now();
        bool hover_success = apoc_control.hoverSwitch(hover_time);
        
        if (hover_success) {
            ROS_INFO("[Step 5/7] Completed hover successfully! Duration: %.1fs",
                     (ros::Time::now() - hover_start_time).toSec());
        } else {
            throw std::runtime_error("Hover operation failed");
        }

        // -------------------------- 步骤6：执行相对飞行到(1,1,1,1.57) --------------------------
        ROS_INFO("\n[Step 6/7] Executing relative flight to (1,1,1,1.57) (Timeout: %.1fs)...", flight_timeout);
        
        // 记录相对飞行开始时间
        ros::Time relative_flight_start_time = ros::Time::now();
        bool relative_flight_success = false;
        
        // 执行相对飞行
        while (ros::ok()) {
            relative_flight_success = apoc_control.flytoRelative(1.0f, 1.0f, 1.0f, 1.57f);
            
            // 检查是否到达目标位置
            if (relative_flight_success) {
                ROS_INFO("[Step 6/7] Successfully reached relative target position!");
                break;
            }
            
            // 检查是否超时
            if ((ros::Time::now() - relative_flight_start_time).toSec() > flight_timeout) {
                throw std::runtime_error("Relative flight timed out");
            }
            
            ros::spinOnce();
            main_rate.sleep();
        }

        // 在相对飞行目标位置悬停一段时间
        ROS_INFO("\n[Step 6/7] Hovering at relative target position for %.1f seconds...", hover_time);
        hover_start_time = ros::Time::now();
        hover_success = apoc_control.hoverSwitch(hover_time);
        
        if (hover_success) {
            ROS_INFO("[Step 6/7] Completed hover at target position! Duration: %.1fs",
                     (ros::Time::now() - hover_start_time).toSec());
        } else {
            throw std::runtime_error("Hover at target position failed");
        }

        // -------------------------- 步骤7：执行降落 --------------------------
        ROS_INFO("\n[Step 7/7] Starting landing sequence (Timeout: %.1fs)...", landing_timeout);
        
        // 记录降落开始时间
        ros::Time land_start_time = ros::Time::now();
        bool land_success = false;
        
        // 执行降落操作
        land_success = apoc_control.landSwitch();
        
        // 检查降落是否成功
        if (land_success) {
            ROS_INFO("[Step 7/7] Landing completed successfully!");
        } else {
            // 检查是否因超时而失败
            if ((ros::Time::now() - land_start_time).toSec() > landing_timeout) {
                throw std::runtime_error("Landing timed out");
            } else {
                throw std::runtime_error("Landing operation failed");
            }
        }

    } catch (const std::exception& e) {
        // 异常处理：打印错误并紧急上锁
        ROS_ERROR("\nTest aborted due to error: %s", e.what());
        apoc_control.armSwitch(0);  // 紧急上锁
        ros::shutdown();
        return -1;
    }

    // 测试完成
    ROS_INFO("\n=== All Flight Test Steps Completed Successfully! ===");

    // 确保无人机已上锁
    if (apoc_control.armSwitch(1)) {  // 检查是否仍处于解锁状态
        apoc_control.armSwitch(0);
        ROS_INFO("Vehicle disarmed after test completion.");
    }

    ros::shutdown();
    return 0;
}
    
