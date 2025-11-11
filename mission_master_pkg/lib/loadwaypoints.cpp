#include"mission_master_pkg/mission_master.h"
/**
 * @brief 航点导入
 */
void MissionMaster::loadWaypoints(){

    TAKEOFF_WAYPOINT = Eigen::Vector3d(TAKEOFF_POSE_X + home_pose.pose.position.x,
                                       TAKEOFF_POSE_Y + home_pose.pose.position.y,
                                       TAKEOFF_POSE_Z + home_pose.pose.position.z);
    PICKUP_START_WAYPOINT = Eigen::Vector3d(PICKUP_START_POSE_X + home_pose.pose.position.x,
                                            PICKUP_START_POSE_Y + home_pose.pose.position.y,
                                            PICKUP_START_POSE_Z + home_pose.pose.position.z);
    PICKUP_END_WAYPOINT = Eigen::Vector3d(PICKUP_END_POSE_X + home_pose.pose.position.x,
                                          PICKUP_END_POSE_Y + home_pose.pose.position.y,
                                          PICKUP_END_POSE_Z + home_pose.pose.position.z);
    AVOID_START_WAYPOINT = Eigen::Vector3d(AVOID_START_POSE_X + home_pose.pose.position.x,
                                           AVOID_START_POSE_Y + home_pose.pose.position.y,
                                           AVOID_START_POSE_Z + home_pose.pose.position.z);
    AVOID_END_WAYPOINT = Eigen::Vector3d(AVOID_END_POSE_X + home_pose.pose.position.x,
                                         AVOID_END_POSE_Y + home_pose.pose.position.y,
                                         AVOID_END_POSE_Z + home_pose.pose.position.z);
    TRACE_START_WAYPOINT = Eigen::Vector3d(TRACE_START_POSE_X + home_pose.pose.position.x,
                                           TRACE_START_POSE_Y + home_pose.pose.position.y,
                                           TRACE_START_POSE_Z + home_pose.pose.position.z);
    TRACE_END_WAYPOINT = Eigen::Vector3d(TRACE_END_POSE_X + home_pose.pose.position.x,
                                         TRACE_END_POSE_Y + home_pose.pose.position.y,
                                         TRACE_END_POSE_Z + home_pose.pose.position.z);
                                        
}