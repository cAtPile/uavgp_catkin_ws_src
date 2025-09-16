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

// 定义逐步降落的步长，单位：米
#define LANDING_STEP 0.1

void apoc::landSwitch(){

    // 记录当前位置作为降落的起始点
    float land_x = current_pose.pose.position.x;
    float land_y = current_pose.pose.position.y;
    float current_z = current_pose.pose.position.z;
    
    // 获取当前偏航角
    tf2::Quaternion quat(
        current_pose.pose.orientation.x,
        current_pose.pose.orientation.y,
        current_pose.pose.orientation.z,
        current_pose.pose.orientation.w
    );
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, land_yaw;
    mat.getRPY(roll, pitch, land_yaw);

    ros::Time total_start = ros::Time::now();
    
    // 计算目标降落高度
    float target_land_z = home_pose.pose.position.z + landing_tolerance_;
    
    // 逐步降低高度，每次下降0.1m
    while (ros::ok() && current_z > target_land_z) {
        // 计算当前目标高度，不低于最终目标高度
        float step_target_z = std::max(current_z - LANDING_STEP, target_land_z);
        
        // 飞向当前步骤的目标高度
        ros::Time step_start = ros::Time::now();
        bool step_reached = false;
        
        // 在当前步骤高度保持位置稳定
        while (ros::ok() && !step_reached && 
               (ros::Time::now() - total_start).toSec() < landing_timeout_) {
            
            flytoAbsolute(land_x, land_y, step_target_z, land_yaw);
            ros::spinOnce();
            rate.sleep();
            
            // 检查是否到达当前步骤的目标高度（考虑一定容差）
            if (fabs(current_pose.pose.position.z - step_target_z) < 0.05) {
                step_reached = true;
                ROS_INFO("Reached intermediate landing altitude: %.2fm", step_target_z);
            }
        }
        
        // 更新当前高度
        current_z = current_pose.pose.position.z;
        
        // 检查是否超时
        if ((ros::Time::now() - total_start).toSec() >= landing_timeout_) {
            ROS_INFO("Landing TIMEOUT, FORCE DISARM");
            armSwitch(0);
        }
    }

    // 到达目标高度后，进行锁定
    if (current_pose.pose.position.z <= target_land_z) {
        armSwitch(0);
    }
}