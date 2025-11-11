#include"mission_master_pkg/mission_master.h"
/**
 * @brief 参数加载
 */
voidMissionMaster::loadParam(){

    //飞行参数
    nh.param("tolerance_waypoint", TOLERANCE_WAYPOINT, 0.10);

    //航点参数
    nh.param("takeoff_pose_x", TAKEOFF_POSE_X, 0.0);
    nh.param("takeoff_pose_y", TAKEOFF_POSE_Y, 0.0);
    nh.param("takeoff_pose_z", TAKEOFF_POSE_Z, 1.0);
    nh.param("pickup_start_pose_x", PICKUP_START_POSE_X, 1.0);
    nh.param("pickup_start_pose_y", PICKUP_START_POSE_Y, 0.0);
    nh.param("pickup_start_pose_z", PICKUP_START_POSE_Z, 1.0);
    nh.param("pickup_end_pose_x", PICKUP_END_POSE_X, 1.0);
    nh.param("pickup_end_pose_y", PICKUP_END_POSE_Y, 1.0);
    nh.param("pickup_end_pose_z", PICKUP_END_POSE_Z, 1.0);
    nh.param("avoid_start_pose_x", AVOID_START_POSE_X, 0.0);
    nh.param("avoid_start_pose_y", AVOID_START_POSE_Y, 1.0);
    nh.param("avoid_start_pose_z", AVOID_START_POSE_Z, 1.0);
    nh.param("avoid_end_pose_x", AVOID_END_POSE_X, 1.0);
    nh.param("avoid_end_pose_y", AVOID_END_POSE_Y, 0.0);
    nh.param("avoid_end_pose_z", AVOID_END_POSE_Z, 1.0);
    nh.param("trace_start_pose_x", TRACE_START_POSE_X, 1.0);
    nh.param("trace_start_pose_y", TRACE_START_POSE_Y, 1.0);
    nh.param("trace_start_pose_z", TRACE_START_POSE_Z, 1.0);
    nh.param("trace_end_pose_x", TRACE_END_POSE_X, 0.0);
    nh.param("trace_end_pose_y", TRACE_END_POSE_Y, 0.0);
    nh.param("trace_end_pose_z", TRACE_END_POSE_Z, 1.0);


}