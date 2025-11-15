#include"mission_master_pkg/mission_master.h"
/**
 * @brief 定点
 */
void MissionMaster::setPoint(std::vector<double> set_point){

    temp_pose.header.frame_id = "map";
    temp_pose.header.stamp = ros::Time::now();
    temp_pose.pose.position.x = set_point[0];
    temp_pose.pose.position.y = set_point[1];
    temp_pose.pose.position.z = set_point[2];
    temp_pose.pose.orientation.x = 0.0;
    temp_pose.pose.orientation.y = 0.0;
    temp_pose.pose.orientation.z = 0.0;
    temp_pose.pose.orientation.w = 1.0;


}

/**
 * @brief 到达检查
 */
bool MissionMaster::reachCheck(std::vector<double> check_point){
    double dx = current_pose.pose.position.x - check_point[0];
    double dy = current_pose.pose.position.y - check_point[1];
    double dz = current_pose.pose.position.z - check_point[2];
    double distance = sqrt(dx * dx + dy * dy + dz * dz);
    return distance < TOLERANCE_WAYPOINT;
}