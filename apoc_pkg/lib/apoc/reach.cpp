#include "apoc_pkg/apoc.h"

bool apoc::reachCheck(float check_x, float check_y, float check_z, float check_yaw) {

    // 距离检查（位置误差）
    float dx = std::abs(current_pose.pose.position.x - check_x);
    float dy = std::abs(current_pose.pose.position.y - check_y);
    float dz = std::abs(current_pose.pose.position.z - check_z);
    float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    bool position_reached = (distance < reach_tolerance_distance_);
    
    /*
    // 角度检查（偏航角误差）
    // 从四元数转换为偏航角（弧度）
    tf2::Quaternion quat(
        current_pose.pose.orientation.x,
        current_pose.pose.orientation.y,
        current_pose.pose.orientation.z,
        current_pose.pose.orientation.w
    );
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    
    // 计算与目标偏航角的差值（考虑角度周期性）
    float dyaw = std::abs(yaw - check_yaw);
    dyaw = std::min(dyaw, 2*static_cast<float>(M_PI) - dyaw);  // 取最小角度差
    bool yaw_reached = (dyaw < reach_tolerance_angle_);

    if(yaw_reached && position_reached){
        return  true;
    }
    */
    return  position_reached;
}
