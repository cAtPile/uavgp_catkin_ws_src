/*
* file:     arm.cpp
* name：    armSwitch
* path：    /lib
* describe：arm/disarm
* input：   arm_key = 0 -> disarm
*           arm_key = 1 -> arm
* output:   true    ->  成功arm/disarm
*           false   ->  失败
* param:    ARMSWITCH_TIMEOUT
* depend:   ros-noetic,cpp,mavros,px4,ununtu20.04,apoc.h
* function: NONE
* vision:   1.0
* method:   调用arm_cmd
* info:     "CANNOT arm/disarm ! DISCONNECT to FCU"
*           因未连接而解锁/上锁失败
*           
*           "Cannot arm/disarm ! not in OFFBOARD mode"
*           因未offboard而解锁/上锁失败
*
*           "Arm/disarm TIMEOUT during"
*           解锁/上锁超时
*
*           "Arm/disarm failed UNKUNOWN reason unexpectedly!"
*           未知原因失败
*   
*           "ROS node is not running ,during arm/disarm"
*           ROS非正常
*/

#include "apoc_pkg/apoc.h"

bool apoc::armSwitch(int arm_key) {

    // 确定目标状态（上锁/解锁）
    bool target_arm_state = (arm_key == 1);
    std::string action = target_arm_state ? "arm" : "disarm";

    // 如果已处于目标状态，直接返回成功
    if (current_state.armed == target_arm_state) {
        return true;
    }

    // 注意：某些情况下解锁可能不需要，但根据注释要求添加此检查
    if (current_state.mode != "OFFBOARD") {
        ROS_WARN("Cannot arm/disarm ! not in OFFBOARD mode");
        return false;
    }

    // 超时设置
    ros::Duration timeout_duration(armswitch_timeout_);
    ros::Time start_time = ros::Time::now();

    // 循环尝试直到状态改变或超时
    while (ros::ok()) {
        // 检查是否已达到目标状态
        if (current_state.armed == target_arm_state) {
            return true;
        }

        // 检查是否超时
        if ((ros::Time::now() - start_time) > timeout_duration) {
            ROS_WARN("Arm/disarm TIMEOUT during");
            return false;
        }

        // 创建上锁/解锁命令
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = target_arm_state;
        arming_client.call(arm_cmd);

        // 处理回调和睡眠
        ros::spinOnce();
        rate.sleep();
    }

    // 检查ROS节点是否正常运行
    if (!ros::ok()) {
        ROS_ERROR("ROS node is not running ,during arm/disarm");
        return false;
    }

    // 如果循环退出但未返回，说明发生未知错误
    ROS_ERROR("Arm/disarm failed UNKUNOWN reason unexpectedly!");
    return false;
}
