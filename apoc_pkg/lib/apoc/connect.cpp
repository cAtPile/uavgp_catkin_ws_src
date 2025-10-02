
/*
* file:     conect.cpp
* path:     /lib
* name:     connectSwitch
* describe: 确保完成连接
* input:    NONE
* output:   true    ->  连接成功
*           false   ->  连接失败
* param:    CONNECT_TIMEOUT
* value:    current_pose
* depend:   ros-noetic,cpp,mavros,px4,ununtu20.04,apoc.h
* function: NONE
* vision:   1.0
* method:   循环等待连接,连接成功持续发布current_pose,防止断开
* info:     "ROS node is not running properly during connect"
*           连接时ros不正常运行
*           
*           "Waiting for FCU connection..."
*           等待连接
*
*           "Failed to connect to FCU , timeout"
*           超时连接失败
*
*           "FCU connection failed unkunown reason unexpectedly!"
*           未知原因失败
*
*/

#include "apoc_pkg/apoc.h"

bool apoc::connectSwitch() {

    // 如果连接成功
    if (current_state.connected) {
        local_pos_pub.publish(current_pose);
        return true;
    }

    ros::Time start_time = ros::Time::now();
    ros::Duration timeout_duration(connect_timeout_);  // 超时10秒

    // 循环等待直到 FCU 连接或者超时
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Waiting for FCU connection...");

        // 检查是否超时
        if ((ros::Time::now() - start_time) > timeout_duration) {
            ROS_ERROR("Failed to connect to FCU , timeout");
            return false;
        }
    }
    
    // 如果连接成功
    if (current_state.connected) {
        local_pos_pub.publish(current_pose);
        return true;
    }

    // 检查ROS节点是否正常运行
    if (!ros::ok()) {
        ROS_ERROR("ROS node is not running properly during connect");
        return false;
    }

    // 额外的安全检查：如果出现异常没有连接成功
    ROS_ERROR("FCU connection failed unkunown reason unexpectedly!");
    return false;
}
