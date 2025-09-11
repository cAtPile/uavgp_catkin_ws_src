#include "apoc_pkg/apoc.h"
#include <ros/ros.h>

const float FLIGHT_TIMEOUT = 30.0f;   // 超时时间 30 秒

int main(int argc, char **argv) {
    // 1. 初始化ROS节点
    ros::init(argc, argv, "apoc_test_node");
    ROS_INFO("=== APOC Test Node Started (Connection + Arm + Offboard) ===");

    // 2. 创建apoc类实例
    apoc apoc_control;

    // 3. 初始化计时器
    ros::Time start_time = ros::Time::now();
    ros::Rate main_rate(10);  // 主循环 10Hz

    try {
        // -------------------------- 步骤1：连接飞控 --------------------------
        ROS_INFO("Step 1: Connecting to FCU...");
        while (ros::ok() && !apoc_control.connectSwitch()) {
            if ((ros::Time::now() - start_time).toSec() > FLIGHT_TIMEOUT) {
                ROS_ERROR("Connect to FCU timed out (exceed %.0fs)", FLIGHT_TIMEOUT);
                return -1;
            }
            ros::spinOnce();
            main_rate.sleep();
        }
        ROS_INFO("Successfully connected to FCU!");

        // -------------------------- 步骤2：解锁无人机 --------------------------
        ROS_INFO("Step 2: Arming vehicle...");
        while (ros::ok() && !apoc_control.armSwitch(1)) {  // arm_key=1 -> 解锁
            if ((ros::Time::now() - start_time).toSec() > FLIGHT_TIMEOUT) {
                ROS_ERROR("Arm vehicle timed out");
                return -1;
            }
            ros::spinOnce();
            main_rate.sleep();
        }
        ROS_INFO("Vehicle armed successfully!");

        // -------------------------- 步骤3：切换到OFFBOARD模式 --------------------------
        ROS_INFO("Step 3: Switching to OFFBOARD mode...");
        while (ros::ok() && !apoc_control.modeSwitch(1)) {  // mode_key=1 -> OFFBOARD
            if ((ros::Time::now() - start_time).toSec() > FLIGHT_TIMEOUT) {
                ROS_ERROR("Switch to OFFBOARD mode timed out");
                return -1;
            }
            ros::spinOnce();
            main_rate.sleep();
        }
        ROS_INFO("Successfully switched to OFFBOARD mode!");

    } catch (const std::exception& e) {
        ROS_ERROR("Test aborted due to exception: %s", e.what());
        apoc_control.armSwitch(0); // 紧急上锁
        return -1;
    }

    ROS_INFO("=== Test Completed: Connection + Arm + Offboard ===");
    ros::shutdown();
    return 0;
}

