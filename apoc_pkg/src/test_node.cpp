#include "apoc_pkg/apoc.h"
#include <ros/ros.h>
#include <stdexcept>  // 用于抛出标准异常

// 从参数服务器读取配置（替代硬编码，提高灵活性）
const std::string FLIGHT_TIMEOUT_PARAM = "flight_timeout";  // 飞行超时参数名
const std::string REACH_TOL_DIST_PARAM = "reach_tolerance_distance";  // 距离到达阈值
const std::string REACH_TOL_ANGLE_PARAM = "reach_tolerance_angle";    // 角度到达阈值

// 默认参数（参数服务器读取失败时使用）
const float DEFAULT_FLIGHT_TIMEOUT = 30.0f;    // 默认飞行超时：30秒
const float DEFAULT_REACH_TOL_DIST = 0.1f;     // 默认距离阈值：0.1米
const float DEFAULT_REACH_TOL_ANGLE = 0.1f;    // 默认角度阈值：0.1弧度

// 测试用绝对目标坐标（可根据需求修改）
const float TARGET_ABS_X = 5.0f;   // 目标X轴绝对坐标（单位：米，基于map坐标系）
const float TARGET_ABS_Y = 2.0f;   // 目标Y轴绝对坐标
const float TARGET_ABS_Z = 2.0f;   // 目标Z轴绝对坐标（飞行高度：2米）
const float TARGET_ABS_YAW = 3.14f; // 目标偏航角（0弧度 = 正前方，基于map坐标系）

int main(int argc, char **argv) {
    // 1. 初始化ROS节点（添加匿名性，避免多节点冲突）
    ros::init(argc, argv, "apoc_abs_flight_test_node", ros::init_options::AnonymousName);
    ROS_INFO("=== APOC Absolute Flight Test Node Started ===");
    ROS_INFO("Target Absolute Position: (X: %.2f, Y: %.2f, Z: %.2f, Yaw: %.2f rad)",
             TARGET_ABS_X, TARGET_ABS_Y, TARGET_ABS_Z, TARGET_ABS_YAW);

    // 2. 创建ROS节点句柄（用于读取参数服务器）
    ros::NodeHandle nh("~");  // 私有命名空间，避免参数冲突

    // 3. 读取配置参数（从launch文件或命令行传入，优先于默认值）
    float flight_timeout, reach_tol_dist, reach_tol_angle;
    if (!nh.getParam(FLIGHT_TIMEOUT_PARAM, flight_timeout)) {
        ROS_WARN("Param '%s' not found, use default: %.1fs", FLIGHT_TIMEOUT_PARAM.c_str(), DEFAULT_FLIGHT_TIMEOUT);
        flight_timeout = DEFAULT_FLIGHT_TIMEOUT;
    }
    if (!nh.getParam(REACH_TOL_DIST_PARAM, reach_tol_dist)) {
        ROS_WARN("Param '%s' not found, use default: %.2fm", REACH_TOL_DIST_PARAM.c_str(), DEFAULT_REACH_TOL_DIST);
        reach_tol_dist = DEFAULT_REACH_TOL_DIST;
    }
    if (!nh.getParam(REACH_TOL_ANGLE_PARAM, reach_tol_angle)) {
        ROS_WARN("Param '%s' not found, use default: %.2frad", REACH_TOL_ANGLE_PARAM.c_str(), DEFAULT_REACH_TOL_ANGLE);
        reach_tol_angle = DEFAULT_REACH_TOL_ANGLE;
    }

    // 4. 创建apoc类实例（核心控制类）
    apoc apoc_control;

    // 5. 初始化主循环频率（10Hz，与飞控通信常用频率）
    ros::Rate main_rate(10);
    ros::Time test_start_time = ros::Time::now();  // 整个测试的起始时间

    try {
        // -------------------------- 步骤1：连接飞控 --------------------------
        ROS_INFO("\n[Step 1/4] Connecting to FCU (Timeout: %.1fs)...", flight_timeout);
        while (ros::ok() && !apoc_control.connectSwitch()) {
            // 检查连接超时
            if ((ros::Time::now() - test_start_time).toSec() > flight_timeout) {
                throw std::runtime_error(
                    "Connect to FCU timed out (exceed " + std::to_string(flight_timeout) + "s)"
                );
            }
            ros::spinOnce();   // 处理回调（如飞控状态更新）
            main_rate.sleep(); // 控制循环频率
        }
        ROS_INFO("[Step 1/4] Successfully connected to FCU!");

        // -------------------------- 步骤2：解锁无人机 --------------------------
        ROS_INFO("\n[Step 2/4] Arming vehicle (Timeout: %.1fs)...", flight_timeout);
        while (ros::ok() && !apoc_control.armSwitch(1)) {  // arm_key=1 → 解锁
            // 检查解锁超时（累计时间不超过总超时）
            if ((ros::Time::now() - test_start_time).toSec() > flight_timeout) {
                throw std::runtime_error("Arm vehicle timed out");
            }
            ros::spinOnce();
            main_rate.sleep();
        }
        ROS_INFO("[Step 2/4] Vehicle armed successfully!");

        // -------------------------- 步骤3：切换OFFBOARD模式 --------------------------
        ROS_INFO("\n[Step 3/4] Switching to OFFBOARD mode (Timeout: %.1fs)...", flight_timeout);
        while (ros::ok() && !apoc_control.modeSwitch(1)) {  // mode_key=1 → OFFBOARD
            // 检查模式切换超时
            if ((ros::Time::now() - test_start_time).toSec() > flight_timeout) {
                throw std::runtime_error("Switch to OFFBOARD mode timed out");
            }
            ros::spinOnce();
            main_rate.sleep();
        }
        ROS_INFO("[Step 3/4] Successfully switched to OFFBOARD mode!");

        // -------------------------- 步骤4：飞往绝对坐标 --------------------------
        ROS_INFO("\n[Step 4/4] Flying to absolute position: (X: %.2f, Y: %.2f, Z: %.2f, Yaw: %.2f rad)",
                 TARGET_ABS_X, TARGET_ABS_Y, TARGET_ABS_Z, TARGET_ABS_YAW);
        ROS_INFO("[Step 4/4] Reach tolerance: Distance=%.2fm, Angle=%.2frad (Timeout: %.1fs)",
                 reach_tol_dist, reach_tol_angle, flight_timeout);

        // 调用绝对飞行接口（传入目标坐标）
        bool reach_success = apoc_control.flytoAbsolute(
            TARGET_ABS_X, TARGET_ABS_Y, TARGET_ABS_Z, TARGET_ABS_YAW
        );

        // 检查飞行结果
        if (reach_success) {
            ROS_INFO("[Step 4/4] Reached target absolute position successfully!");
        } else {
            throw std::runtime_error("Failed to reach target position (timeout or error)");
        }

    } catch (const std::exception& e) {
        // 异常处理：打印错误并紧急上锁
        ROS_ERROR("\nTest aborted due to error: %s", e.what());
        apoc_control.armSwitch(0);  // arm_key=0 → 紧急上锁
        ros::shutdown();
        return -1;
    }

    // -------------------------- 测试完成 --------------------------
    ROS_INFO("\n=== All Test Steps Completed Successfully! ===");

    // （可选）测试后自动上锁（根据需求选择是否启用）
    // apoc_control.armSwitch(0);
    // ROS_INFO("Vehicle disarmed automatically after test.");

    ros::shutdown();
    return 0;
}
