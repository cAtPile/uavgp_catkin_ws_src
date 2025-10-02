
#include "apoc_pkg/apoc.h"

bool apoc::takeoffSwitch(float takeoff_alt) {
    // 检查是否已解锁，未解锁则无法起飞
    if (!current_state.armed) {
        ROS_WARN("Cannot takeoff: Vehicle is not armed");
        return false;
    }

    home_pose = current_pose;
    ROS_INFO("Home position recorded");

    float takeoff_x = current_pose.pose.position.x;
    float takeoff_y = current_pose.pose.position.y;
    float takeoff_z = current_pose.pose.position.z + takeoff_alt;
    
    // 保持当前偏航角
    tf2::Quaternion quat(
        current_pose.pose.orientation.x,
        current_pose.pose.orientation.y,
        current_pose.pose.orientation.z,
        current_pose.pose.orientation.w
    );
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, takeoff_yaw;
    mat.getRPY(roll, pitch, takeoff_yaw);

    ROS_INFO_STREAM("Taking off to altitude: " << takeoff_z << "m");

    // 使用相对飞行函数飞到目标高度
    if (flytoAbsolute(takeoff_x, takeoff_y, takeoff_z, takeoff_yaw)) {
        ROS_INFO("Ttakeoff succeed");
        return true;
    }else{
        ROS_ERROR("Failed to execute takeoff movement");
        return false;
    }

    ROS_ERROR("Takeoff failed due to ROS node shutdown");
    return false;
}
