#include "apoc_pkg/apoc.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <thread>

// 1. 定义全局可用的常量（放在函数外或函数最顶部，确保所有循环都能访问）
#define LANDING_STEP 0.2f          // 逐步降落步长
const int CMD_RETRY_COUNT = 3;     // 指令重试次数
const double STEP_TIMEOUT = 5.0;   // 单步降落超时时间
const double FINAL_STEP_TIMEOUT = 8.0; // 最后一步（飞回Home高度）超时时间
const float FINAL_HEIGHT_TOL = 0.03f;  // 最后一步高度容差
const float HEIGHT_PRECISION = 0.01f;  // 通用高度精度

void apoc::landSwitch() {

    // 记录降落起始点
    float land_x = current_pose.pose.position.x;
    float land_y = current_pose.pose.position.y;
    float current_z = current_pose.pose.position.z;
    
    // 解析当前偏航角
    tf2::Quaternion quat(
        current_pose_copy.pose.orientation.x,
        current_pose_copy.pose.orientation.y,
        current_pose_copy.pose.orientation.z,
        current_pose_copy.pose.orientation.w
    );
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, land_yaw;
    mat.getRPY(roll, pitch, land_yaw);

    // 初始化总降落计时器
    ros::Time total_start = ros::Time::now();

    // 计算目标降落高度
    float target_land_z = home_base_z + landing_tolerance_;
    float home_base_z = home_pose.pose.position.z; // 单独存储Home基准高度，用于最后一步精准降落

    //高度有效性校验
    if (target_land_z < 0.0f) {
        ROS_WARN("Invalid target landing height (%.2fm), reset to 0.0m", target_land_z);
        target_land_z = 0.0f;
        home_base_z = 0.0f; // 同步重置Home基准高度，确保一致性
    }

    // 1. 逐步降落主循环
    while (ros::ok() && (current_z - target_land_z) > HEIGHT_PRECISION) {
        // 计算当前步骤目标高度（确保不低于最终容忍高度）
        float step_target_z = std::max(current_z - LANDING_STEP, target_land_z);
        
        // 初始化单步降落状态
        ros::Time step_start = ros::Time::now();
        bool step_reached = false;
        
        // 2. 单步高度稳定循环（确保到达当前步骤高度后再继续下降）
        while (ros::ok() && !step_reached) {
            // 2.1 检查单步超时（优先处理，避免长时间等待）
            if ((ros::Time::now() - step_start).toSec() >= STEP_TIMEOUT) {
                ROS_WARN("Step landing timeout (target: %.2fm), skip to next step", step_target_z);
                break;
            }
            // 2.2 检查总降落超时（超过设定时间强制结束）
            if ((ros::Time::now() - total_start).toSec() >= landing_timeout_) {
                ROS_ERROR("Total landing timeout (%.1fs), force disarm", landing_timeout_);
                armSwitch(0);
                return;
            }
            
            // 2.3 发送飞控指令（带重试机制，确保指令可靠）
            bool cmd_sent = false;
            for (int retry = 0; retry < CMD_RETRY_COUNT; ++retry) { // 此处使用全局定义的CMD_RETRY_COUNT
                if (flytoAbsolute(land_x, land_y, step_target_z, land_yaw)) {
                    cmd_sent = true;
                    break;
                }
                ROS_WARN("Flyto command failed (retry %d/%d), wait 500ms", retry + 1, CMD_RETRY_COUNT);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            if (!cmd_sent) {
                ROS_ERROR("Flyto command failed after %d retries, force disarm", CMD_RETRY_COUNT);
                armSwitch(0);
                return;
            }

            // 处理回调并控制循环频率
            ros::spinOnce();
            rate.sleep();

            // 2.4 检查是否到达当前步骤高度（加锁读取最新位置）
            geometry_msgs::PoseStamped current_pose_check;
            {
                std::lock_guard<std::mutex> lock(current_pose_mutex_);
                current_pose_check = current_pose;
            }
            if (fabs(current_pose_check.pose.position.z - step_target_z) < 0.05f) {
                step_reached = true;
                ROS_INFO("Reached intermediate altitude: %.2fm (current: %.2fm)", 
                         step_target_z, current_pose_check.pose.position.z);
            }
        }
            
        // 3. 更新当前高度（加锁读取最新值，避免使用旧数据）
        {
            std::lock_guard<std::mutex> lock(current_pose_mutex_);
            current_z = current_pose.pose.position.z;
        }
        
        // 再次检查总超时（双重保障）
        if ((ros::Time::now() - total_start).toSec() >= landing_timeout_) {
            ROS_ERROR("Total landing timeout, force disarm");
            armSwitch(0);
            break;
        }
    }

    // 4. 最后一步：精准飞回Home基准高度（你的核心新增逻辑）
    ROS_INFO("Start final step: land to home base height (%.2fm)", home_base_z);
    ros::Time final_step_start = ros::Time::now();
    bool home_height_reached = false;

    while (ros::ok() && !home_height_reached) {
        // 4.1 检查最后一步超时（避免无限等待）
        if ((ros::Time::now() - final_step_start).toSec() >= FINAL_STEP_TIMEOUT) {
            ROS_WARN("Final step timeout (target home height: %.2fm), proceed to disarm", home_base_z);
            break;
        }
        // 4.2 检查总降落超时（确保不超过全局超时）
        if ((ros::Time::now() - total_start).toSec() >= landing_timeout_) {
            ROS_ERROR("Total landing timeout during final step, force disarm");
            armSwitch(0);
            return;
        }

        // 4.3 发送最后一步飞控指令（带重试，使用全局CMD_RETRY_COUNT）
        bool final_cmd_sent = false;
        for (int retry = 0; retry < CMD_RETRY_COUNT; ++retry) {
            if (flytoAbsolute(land_x, land_y, home_base_z, land_yaw)) {
                final_cmd_sent = true;
                break;
            }
            ROS_WARN("Final flyto command failed (retry %d/%d), wait 500ms", retry + 1, CMD_RETRY_COUNT);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        if (!final_cmd_sent) {
            ROS_ERROR("Final flyto command failed after %d retries, force disarm", CMD_RETRY_COUNT);
            armSwitch(0);
            return;
        }

        // 处理回调并控制频率
        ros::spinOnce();
        rate.sleep();

        // 4.4 检查是否到达Home基准高度（加锁读取最新位置）
        geometry_msgs::PoseStamped current_pose_final;
        {
            std::lock_guard<std::mutex> lock(current_pose_mutex_);
            current_pose_final = current_pose;
        }
        if (fabs(current_pose_final.pose.position.z - home_base_z) < FINAL_HEIGHT_TOL) {
            home_height_reached = true;
            ROS_INFO("Reached home base height: %.2fm (current: %.2fm)", 
                     home_base_z, current_pose_final.pose.position.z);
        }
    }

    // 5. 最终解锁判断（基于实际高度，确保安全）
    geometry_msgs::PoseStamped current_pose_disarm;
    {
        std::lock_guard<std::mutex> lock(current_pose_mutex_);
        current_pose_disarm = current_pose;
    }

    // 判定是否成功降落（优先以Home基准高度为准）
    if (fabs(current_pose_disarm.pose.position.z - home_base_z) < FINAL_HEIGHT_TOL) {
        ROS_INFO("Landing successful! Final height: %.2fm (home base: %.2fm), disarm", 
                 current_pose_disarm.pose.position.z, home_base_z);
        armSwitch(0);
    } else if (current_pose_disarm.pose.position.z <= target_land_z + HEIGHT_PRECISION) {
        ROS_WARN("Landed to tolerance height (%.2fm) but not home base (%.2fm), disarm", 
                 current_pose_disarm.pose.position.z, home_base_z);
        armSwitch(0);
    } else {
        ROS_ERROR("Final height (%.2fm) exceeds target (%.2fm), force disarm", 
                  current_pose_disarm.pose.position.z, target_land_z);
        armSwitch(0);
    }
}