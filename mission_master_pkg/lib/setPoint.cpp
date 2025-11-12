#include"mission_master_pkg/mission_master.h"
/**
 * @brief 定点
 */
void MissionMaster::setPoint(Eigen::Vector3d set_point){

    geometry_msgs::PoseStamped temp_pose;
    temp_pose.header.frame_id = "map";
    temp_pose.header.stamp = ros::Time::now();
    temp_pose.pose.position.x = pose_v3d.x();
    temp_pose.pose.position.y = pose_v3d.y();
    temp_pose.pose.position.z = pose_v3d.z();
    temp_pose.pose.orientation.x = 0.0;
    temp_pose.pose.orientation.y = 0.0;
    temp_pose.pose.orientation.z = 0.0;
    temp_pose.pose.orientation.w = 1.0;

    //local_pos_pub.publish(temp_pose);
}