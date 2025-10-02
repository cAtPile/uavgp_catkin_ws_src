/*
* file:     land.cpp
* name：    landSwitch
* describe：降落
* input：   NONE
* output:   true    ->  降落成功
*           false   ->  降落失败
* param:    LANDING_TIMEOUT,LANDING_TOLERANCE
* depend:   ros-noetic,cpp,mavros,px4,ununtu20.04,apoc.h
* function: armSwitch
* vision:   1.0
* method：  记录当前位置，使用flytoAbsolute/flytoPIDcorrect到当前x/y/LANDING_TOLERANCE/yaw,然后disarm
* info:     
*/
#include "apoc_pkg/apoc.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <thread>

//逐步降落的步长，单位：米
#define LANDING_STEP 0.2f

void apoc::landSwitch() {
    // 读取当前位置
    geometry_msgs::PoseStamped current_pose_copy;
    {
        std::lock_guard<std::mutex> lock(current_pose_mutex_); 
        current_pose_copy = current_pose; 
    }

    // 记录降落起始点
    float land_x = current_pose_copy.pose.position.x;
    float land_y = current_pose_copy.pose.position.y;
    float current_z = current_pose_copy.pose.position.z;
    
    //解析当前偏航角
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
    
    /*
    // 3. 计算目标降落高度（加home_pose互斥锁，避免数据竞争）
    float target_land_z;
    {
        std::lock_guard<std::mutex> lock(home_pose_mutex_); // 需在apoc类头文件中声明该锁
        target_land_z = home_pose.pose.position.z + landing_tolerance_;
    }
    */
    // 计算目标降落高度
    float target_land_z = home_pose.pose.position.z + landing_tolerance_;

    // 目标高度有效性校验
    if (target_land_z < 0.0f) {
        ROS_WARN("Invalid target landing height (%.2fm), reset to 0.0m", target_land_z);
        target_land_z = 0.0f;
    }

    // 4. 逐步降落主循环
    while (ros::ok() && (current_z - target_land_z) > 0.01f) {

        // 计算当前步骤目标高度
        float step_target_z = std::max(current_z - LANDING_STEP, target_land_z);
        
        // 初始化单步降落状态
        ros::Time step_start = ros::Time::now();
        bool step_reached = false;
        const int CMD_RETRY_COUNT = 3; // 指令重试次数（固定值，不新增宏）
        const double STEP_TIMEOUT = 5.0; // 单步超时时间（5秒，避免卡在某一高度）
        
        // 5. 单步高度稳定循环（确保到达当前步骤高度后再继续下降）
        while (ros::ok() && !step_reached) {
            // 5.1 检查单步超时（优先处理，避免长时间等待）
            if ((ros::Time::now() - step_start).toSec() >= STEP_TIMEOUT) {
                ROS_WARN("Step landing timeout (target: %.2fm), skip to next step", step_target_z);
                break;
            }
            // 5.2 检查总降落超时（超过设定时间强制结束）
            if ((ros::Time::now() - total_start).toSec() >= landing_timeout_) {
                ROS_ERROR("Total landing timeout (%.1fs), force disarm", landing_timeout_);
                armSwitch(0);
                return; // 直接退出函数，避免后续无效逻辑
            }
            
            // 5.3 发送飞控指令（带重试机制，确保指令发送成功）
            bool cmd_sent = false;
            for (int retry = 0; retry < CMD_RETRY_COUNT; ++retry) {
                if (flytoAbsolute(land_x, land_y, step_target_z, land_yaw)) {
                    cmd_sent = true;
                    break; // 指令发送成功，退出重试
                }
                ROS_WARN("Flyto command failed (retry %d/%d), wait 500ms", retry + 1, CMD_RETRY_COUNT);
                std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 重试间隔
            }
            // 指令重试多次失败，强制解锁
            if (!cmd_sent) {
                ROS_ERROR("Flyto command failed after %d retries, force disarm", CMD_RETRY_COUNT);
                armSwitch(0);
                return;
            }

            // 处理ROS回调（更新位置、状态等），控制循环频率
            ros::spinOnce();
            rate.sleep();

            // 5.4 检查是否到达当前步骤高度（加锁读取最新位置）
            geometry_msgs::PoseStamped current_pose_check;
            {
                std::lock_guard<std::mutex> lock(current_pose_mutex_);
                current_pose_check = current_pose;
            }
            // 高度容差0.05m（与原有逻辑一致，确保稳定后再进入下一步）
            if (fabs(current_pose_check.pose.position.z - step_target_z) < 0.05f) {
                step_reached = true;
                ROS_INFO("Reached intermediate landing altitude: %.2fm (current: %.2fm)", 
                         step_target_z, current_pose_check.pose.position.z);
            }
        }
            
        // 6. 更新当前高度（加锁读取最新值，避免使用旧数据）
        {
            std::lock_guard<std::mutex> lock(current_pose_mutex_);
            current_z = current_pose.pose.position.z;
        }
        
        // 再次检查总超时（双重保障，避免循环内超时未触发）
        if ((ros::Time::now() - total_start).toSec() >= landing_timeout_) {
            ROS_ERROR("Total landing timeout, force disarm");
            armSwitch(0);
            break;
        }
    }

    // 7. 最终解锁判断（确保到达目标高度后再解锁）
    geometry_msgs::PoseStamped current_pose_disarm;
    {
        std::lock_guard<std::mutex> lock(current_pose_mutex_);
        current_pose_disarm = current_pose;
    }

    // 到达目标高度（或低于），执行解锁
    if (current_pose_disarm.pose.position.z <= target_land_z + 0.01f) { // 容差处理，避免浮点数误差
        ROS_INFO("Landing successful (final height: %.2fm), disarm", current_pose_disarm.pose.position.z);
        armSwitch(0);
    } else {
        ROS_WARN("Final height (%.2fm) exceeds target (%.2fm), still disarm", 
                 current_pose_disarm.pose.position.z, target_land_z);
        armSwitch(0); // 即使未完全达标，也强制解锁（避免悬停耗电或失控）
    }
}