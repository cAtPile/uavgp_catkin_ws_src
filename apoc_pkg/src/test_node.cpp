#include "apoc_pkg/apoc.h"
#include <ros/ros.h>

// 测试参数配置（可根据实际场景调整）
const float TAKEOFF_ALTITUDE = 1.0f;       // 起飞高度（1米）
const float TARGET_X = 2.0f;               // 目标位置X（相对起飞点前2米）
const float TARGET_Y = 0.0f;               // 目标位置Y（相对起飞点无偏移）
const float TARGET_Z = 1.0f;               // 目标位置Z（保持1米高度）
const float TARGET_YAW = 0.0f;             // 目标偏航角（0弧度，正前方）
const float HOVER_DURATION = 3.0f;         // 悬停时间（3秒）
const float FLIGHT_TIMEOUT = 60.0f;        // 整体飞行超时（60秒）

int main(int argc, char **argv) {
    // 1. 初始化ROS节点
    ros::init(argc, argv, "apoc_test_node");
    ROS_INFO("=== APOC Test Node Started ===");

    // 2. 创建apoc类实例（初始化ROS句柄、订阅/发布器等）
    apoc apoc_control;

    // 3. 初始化超时计时器
    ros::Time start_time = ros::Time::now();
    ros::Rate main_rate(10);  // 主循环频率（10Hz）

    // 4. 核心测试流程
    try {
        // -------------------------- 步骤1：连接飞控 --------------------------
        ROS_INFO("Step 1: Connecting to FCU...");
        while (ros::ok() && !apoc_control.connectSwitch()) {
            // 检查整体超时
            if ((ros::Time::now() - start_time).toSec() > FLIGHT_TIMEOUT) {
                ROS_ERROR("Connect to FCU timed out (exceed %ds)", (int)FLIGHT_TIMEOUT);
                return -1;
            }
            ros::spinOnce();
            main_rate.sleep();
        }
        ROS_INFO("Successfully connected to FCU!");

        // -------------------------- 步骤2：切换到OFFBOARD模式 --------------------------
        ROS_INFO("Step 2: Switching to OFFBOARD mode...");
        while (ros::ok() && !apoc_control.modeSwitch(1)) {  // mode_key=1 -> OFFBOARD
            if ((ros::Time::now() - start_time).toSec() > FLIGHT_TIMEOUT) {
                ROS_ERROR("Switch to OFFBOARD mode timed out");
                return -1;
            }
            ros::spinOnce();
            main_rate.sleep();
        }
        ROS_INFO("Successfully switched to OFFBOARD mode!");

        // -------------------------- 步骤3：解锁无人机 --------------------------
        ROS_INFO("Step 3: Arming vehicle...");
        while (ros::ok() && !apoc_control.armSwitch(1)) {  // arm_key=1 -> 解锁
            if ((ros::Time::now() - start_time).toSec() > FLIGHT_TIMEOUT) {
                ROS_ERROR("Arm vehicle timed out");
                return -1;
            }
            ros::spinOnce();
            main_rate.sleep();
        }
        ROS_INFO("Vehicle armed successfully!");

        // -------------------------- 步骤4：起飞到目标高度 --------------------------
        ROS_INFO("Step 4: Taking off to %.2f meters...", TAKEOFF_ALTITUDE);
        if (!apoc_control.takeoffSwitch(TAKEOFF_ALTITUDE)) {
            ROS_ERROR("Takeoff failed!");
            // 起飞失败时尝试上锁
            apoc_control.armSwitch(0);
            return -1;
        }
        ROS_INFO("Takeoff completed! Current altitude: %.2f meters", TAKEOFF_ALTITUDE);

        // -------------------------- 步骤5：悬停指定时间 --------------------------
        ROS_INFO("Step 5: Hovering for %.1f seconds...", HOVER_DURATION);
        if (!apoc_control.hoverSwitch(HOVER_DURATION)) {
            ROS_WARN("Hovering interrupted, continue to next step...");
        }

        // -------------------------- 步骤6：PID控制飞往目标位置 --------------------------
        ROS_INFO("Step 6: Flying to target position via PID control...");
        ROS_INFO("Target: [X:%.2f, Y:%.2f, Z:%.2f, Yaw:%.2f rad]", 
                 TARGET_X, TARGET_Y, TARGET_Z, TARGET_YAW);
        
        while (ros::ok() && !apoc_control.flytoPIDcorrect(TARGET_X, TARGET_Y, TARGET_Z, TARGET_YAW)) {
            if ((ros::Time::now() - start_time).toSec() > FLIGHT_TIMEOUT) {
                ROS_ERROR("PID flight to target timed out");
                // 超时后尝试降落
                apoc_control.landSwitch();
                return -1;
            }
            ros::spinOnce();
            main_rate.sleep();
        }
        ROS_INFO("Successfully reached target position!");

        // -------------------------- 步骤7：目标位置悬停 --------------------------
        ROS_INFO("Step 7: Hovering at target position for %.1f seconds...", HOVER_DURATION);
        if (!apoc_control.hoverSwitch(HOVER_DURATION)) {
            ROS_WARN("Hovering at target interrupted, starting landing...");
        }

        // -------------------------- 步骤8：降落并上锁 --------------------------
        ROS_INFO("Step 8: Initiating landing sequence...");
        if (!apoc_control.landSwitch()) {
            ROS_ERROR("Landing failed! Forcing disarm...");
            apoc_control.armSwitch(0);
            return -1;
        }
        ROS_INFO("Landing completed successfully! Vehicle disarmed.");

    } catch (const std::exception& e) {
        // 异常处理（如ROS节点异常、飞控断开）
        ROS_ERROR("Test process aborted due to exception: %s", e.what());
        // 紧急上锁
        apoc_control.armSwitch(0);
        return -1;
    }

    // 5. 测试完成
    ROS_INFO("=== All Test Steps Completed Successfully ===");
    ros::shutdown();
    return 0;
}