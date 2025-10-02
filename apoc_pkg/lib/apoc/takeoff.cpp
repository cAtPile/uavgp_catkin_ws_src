
#include "apoc_pkg/apoc.h"

bool apoc::takeoffSwitch(float takeoff_alt) {
    // 检查是否已解锁，未解锁则无法起飞
    if (!current_state.armed) {
        ROS_WARN("Cannot takeoff: Vehicle is not armed");
        return false;
    }

    //互斥锁
    geometry_msgs::PoseStamped current_pose_copy;
    {
        std::lock_guard<std::mutex> lock(current_pose_mutex_); // 读操作加锁
        current_pose_copy = current_pose; // 拷贝到局部变量
    }

    //写互斥锁
    {
        std::lock_guard<std::mutex> lock(current_pose_mutex_);
        home_pose = current_pose_copy;
    }
    ROS_INFO("Home position recorded");

    // 计算起飞目标位置（在当前位置基础上升高到指定高度）
    float takeoff_x = current_pose_copy.pose.position.x;
    float takeoff_y = current_pose_copy.pose.position.y;
    float takeoff_z = current_pose_copy.pose.position.z + takeoff_alt;
    
    // 保持当前偏航角
    tf2::Quaternion quat(
        current_pose_copy.pose.orientation.x,
        current_pose_copy.pose.orientation.y,
        current_pose_copy.pose.orientation.z,
        current_pose_copy.pose.orientation.w
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
