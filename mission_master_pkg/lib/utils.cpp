#include"mission_master_pkg/mission_master.h"
/**
 * @brief 定点
 */
void MissionMaster::setPoint(Eigen::Vector3d set_point){

    temp_pose.header.frame_id = "map";
    temp_pose.header.stamp = ros::Time::now();
    temp_pose.pose.position.x = pose_v3d.x();
    temp_pose.pose.position.y = pose_v3d.y();
    temp_pose.pose.position.z = pose_v3d.z();
    temp_pose.pose.orientation.x = 0.0;
    temp_pose.pose.orientation.y = 0.0;
    temp_pose.pose.orientation.z = 0.0;
    temp_pose.pose.orientation.w = 1.0;


}

/**
 * @brief 到达检查
 */
bool MissionMaster::reachCheck(Eigen::Vector3d check_point){
    double dx = current_pose.pose.position.x - check_point.x();
    double dy = current_pose.pose.position.y - check_point.y();
    double dz = current_pose.pose.position.z - check_point.z();
    double distance = sqrt(dx * dx + dy * dy + dz * dz);
    return distance < TOLERANCE_WAYPOINT;
}