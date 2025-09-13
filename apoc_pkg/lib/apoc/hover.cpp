/*
* file:     hover.cpp
* name：    hoverSwitch
* path:     /lib
* describe：悬停（使用PID控制维持当前位置）
* input：   hover_time - 悬停时长(秒)
* output:   true    ->  悬停成功
*           false   ->  悬停失败
* param:    NONE
* depend:   ros-noetic,cpp,mavros,px4,ununtu20.04,apoc.h
* function: flytoPIDcorrect, reachCheck
* vision:   2.0
* method：  通过flytoPIDcorrect()持续发布当前位置作为目标，维持悬停
* info:     "ROS node shutdown during hover" - 盘旋时ros节点异常
*/

#include "apoc_pkg/apoc.h"

bool apoc::hoverSwitch(float hover_time) {
    // 检查无人机是否已武装
    if (!current_state.armed) {
        ROS_WARN("Cannot hover: Vehicle is not armed");
        return false;
    }

    // 获取悬停起始位置和姿态（作为目标位置）
    float target_x = current_pose.pose.position.x;
    float target_y = current_pose.pose.position.y;
    float target_z = current_pose.pose.position.z;

    // 处理偏航角周期性（转换为[-π, π]范围）
    tf2::Quaternion current_quat(
        current_pose.pose.orientation.x,
        current_pose.pose.orientation.y,
        current_pose.pose.orientation.z,
        current_pose.pose.orientation.w
    );
    tf2::Matrix3x3 current_mat(current_quat);
    double roll, pitch, target_yaw;
    current_mat.getRPY(roll, pitch, target_yaw);
    // 归一化偏航角
    target_yaw = fmod(target_yaw, 2 * M_PI);
    if (target_yaw > M_PI) target_yaw -= 2 * M_PI;
    else if (target_yaw < -M_PI) target_yaw += 2 * M_PI;

    ROS_INFO_STREAM("Start hovering at [X:" << target_x 
                  << ", Y:" << target_y << ", Z:" << target_z 
                  << ", Yaw:" << target_yaw << " rad] for " << hover_time << "s");

    // 初始化计时器
    ros::Time start_time = ros::Time::now();
    ros::Rate check_rate(10);  // 检查频率（10Hz）

    // 悬停主循环：持续调用PID控制维持在目标位置
    while (ros::ok()) {
        // 检查悬停时间是否已到
        float elapsed = (ros::Time::now() - start_time).toSec();
        if (elapsed >= hover_time) {
            ROS_INFO("Hover completed successfully (duration: %.2fs)", elapsed);
            return true;
        }

        // 调用PID控制函数，以当前位置为目标（维持悬停）
        // 注意：flytoPIDcorrect返回true表示"到达目标"，但悬停时我们需要持续调用
        bool reached = flytoPIDcorrect(target_x, target_y, target_z, target_yaw);
        
        // 若PID控制返回false（如超时或异常），则悬停失败
        if (!reached) {
            ROS_WARN("Hover failed: PID control returned error");
            return false;
        }

        // 短暂休眠，避免循环过于频繁
        ros::spinOnce();
        check_rate.sleep();
    }

    // ROS节点异常退出
    ROS_ERROR("ROS node shutdown during hover");
    return false;
}
