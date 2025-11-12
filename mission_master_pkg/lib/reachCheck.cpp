#include"mission_master_pkg/mission_master.h"
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