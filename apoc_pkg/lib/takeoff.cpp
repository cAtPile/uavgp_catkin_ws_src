/*
* name：    takeoffSwitch
* file:     takeoff.cpp
* path:     /lib
* describe：起飞，并记录home位置
* input：   takeoff_alt
* output:   true    ->  起飞
*           false   ->  无法起飞
* param:    NONE
* depend:   ros-noetic,cpp,mavros,px4,ununtu20.04,apoc.h
* function: reachCheck，flytoAbsolute/flytoPIDcorrect
* vision:   1.0
* method：  记录起飞前位置和home位置，加上起飞高度，使用flytoRelative到目的地
* info:     
*/
#include "apoc_pkg/apoc.h"

bool apoc::takeoffSwitch(float takeoff_alt) {
    // 检查是否已解锁，未解锁则无法起飞
    if (!current_state.armed) {
        ROS_WARN("Cannot takeoff: Vehicle is not armed");
        return false;
    }

    // 记录当前位置作为home位置
    home_pose = current_position;
    ROS_INFO("Home position recorded");

    // 计算起飞目标位置（在当前位置基础上升高到指定高度）
    float takeoff_x = current_position.pose.position.x;
    float takeoff_y = current_position.pose.position.y;
    float takeoff_z = current_position.pose.position.z + takeoff_alt;
    
    // 保持当前偏航角
    tf2::Quaternion quat(
        current_position.pose.orientation.x,
        current_position.pose.orientation.y,
        current_position.pose.orientation.z,
        current_position.pose.orientation.w
    );
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, takeoff_yaw;
    mat.getRPY(roll, pitch, takeoff_yaw);

    ROS_INFO_STREAM("Taking off to altitude: " << takeoff_z << "m");

    // 使用相对飞行函数飞到目标高度
    if (!flytoRelative(takeoff_x, takeoff_y, takeoff_z, takeoff_yaw)) {
        ROS_ERROR("Failed to execute takeoff movement");
        return false;
    }

    ROS_ERROR("Takeoff failed due to ROS node shutdown");
    return false;
}