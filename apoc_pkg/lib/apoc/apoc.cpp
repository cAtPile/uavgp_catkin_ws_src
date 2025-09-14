
/*
*   file：      apoc.c
*   name:       apoc.c
*   discribe:   apoc构造函数
*               ros话题初始化，变量初始化
*   vision:     1.0
*   path:
*/

#include "apoc_pkg/apoc.h"

apoc::apoc(): rate(20.0){

     nh.param("apoc_pkg/connect_timeout", connect_timeout_, 10.0);
     ROS_INFO("connect_timeout: %.1fs", connect_timeout_);
     nh.param("apoc_pkg/modeswitch_timeout", modeswitch_timeout_, 10.0);
     ROS_INFO("modeswitch_timeout: %.1fs", modeswitch_timeout_);
     nh.param("apoc_pkg/armswitch_timeout", armswitch_timeout_, 10.0);
     ROS_INFO("armswitch_timeout: %.1fs", armswitch_timeout_);
     nh.param("apoc_pkg/fly_ab_timeout", fly_ab_timeout_, 30.0);
     ROS_INFO("fly_ab_timeout: %.1fs", fly_ab_timeout_);
     nh.param("apoc_pkg/reach_tolerance_distance", reach_tolerance_distance_, 0.1);
     ROS_INFO("reach_tolerance_distance: %.1fm", reach_tolerance_distance_);
     nh.param("apoc_pkg/reach_tolerance_angle", reach_tolerance_angle_, 0.1);
     ROS_INFO("reach_tolerance_angle: %.1frad", reach_tolerance_angle_);
     
         // ============== PID参数获取（新增） ==============
    // X轴PID
    nh.param("apoc_pkg/PID_X_KP", pid_x_kp_, 2.5);
    ROS_INFO("PID_X_KP: %.2f", pid_x_kp_);
    nh.param("apoc_pkg/PID_X_KI", pid_x_ki_, 0.1);
    ROS_INFO("PID_X_KI: %.2f", pid_x_ki_);
    nh.param("apoc_pkg/PID_X_KD", pid_x_kd_, 0.05);
    ROS_INFO("PID_X_KD: %.2f", pid_x_kd_);
    nh.param("apoc_pkg/PID_X_OUT_MIN", pid_x_out_min_, -1.0);
    ROS_INFO("PID_X_OUT_MIN: %.2f", pid_x_out_min_);
    nh.param("apoc_pkg/PID_X_OUT_MAX", pid_x_out_max_, 1.0);
    ROS_INFO("PID_X_OUT_MAX: %.2f", pid_x_out_max_);
    nh.param("apoc_pkg/PID_X_INT_MIN", pid_x_int_min_, -0.5);
    ROS_INFO("PID_X_INT_MIN: %.2f", pid_x_int_min_);
    nh.param("apoc_pkg/PID_X_INT_MAX", pid_x_int_max_, 0.5);
    ROS_INFO("PID_X_INT_MAX: %.2f", pid_x_int_max_);

    // Y轴PID
    nh.param("apoc_pkg/PID_Y_KP", pid_y_kp_, 2.5);
    ROS_INFO("PID_Y_KP: %.2f", pid_y_kp_);
    nh.param("apoc_pkg/PID_Y_KI", pid_y_ki_, 0.1);
    ROS_INFO("PID_Y_KI: %.2f", pid_y_ki_);
    nh.param("apoc_pkg/PID_Y_KD", pid_y_kd_, 0.05);
    ROS_INFO("PID_Y_KD: %.2f", pid_y_kd_);
    nh.param("apoc_pkg/PID_Y_OUT_MIN", pid_y_out_min_, -1.0);
    ROS_INFO("PID_Y_OUT_MIN: %.2f", pid_y_out_min_);
    nh.param("apoc_pkg/PID_Y_OUT_MAX", pid_y_out_max_, 1.0);
    ROS_INFO("PID_Y_OUT_MAX: %.2f", pid_y_out_max_);
    nh.param("apoc_pkg/PID_Y_INT_MIN", pid_y_int_min_, -0.5);
    ROS_INFO("PID_Y_INT_MIN: %.2f", pid_y_int_min_);
    nh.param("apoc_pkg/PID_Y_INT_MAX", pid_y_int_max_, 0.5);
    ROS_INFO("PID_Y_INT_MAX: %.2f", pid_y_int_max_);

    // Z轴PID
    nh.param("apoc_pkg/PID_Z_KP", pid_z_kp_, 3.0);
    ROS_INFO("PID_Z_KP: %.2f", pid_z_kp_);
    nh.param("apoc_pkg/PID_Z_KI", pid_z_ki_, 0.2);
    ROS_INFO("PID_Z_KI: %.2f", pid_z_ki_);
    nh.param("apoc_pkg/PID_Z_KD", pid_z_kd_, 0.1);
    ROS_INFO("PID_Z_KD: %.2f", pid_z_kd_);
    nh.param("apoc_pkg/PID_Z_OUT_MIN", pid_z_out_min_, -0.8);
    ROS_INFO("PID_Z_OUT_MIN: %.2f", pid_z_out_min_);
    nh.param("apoc_pkg/PID_Z_OUT_MAX", pid_z_out_max_, 0.8);
    ROS_INFO("PID_Z_OUT_MAX: %.2f", pid_z_out_max_);
    nh.param("apoc_pkg/PID_Z_INT_MIN", pid_z_int_min_, -0.4);
    ROS_INFO("PID_Z_INT_MIN: %.2f", pid_z_int_min_);
    nh.param("apoc_pkg/PID_Z_INT_MAX", pid_z_int_max_, 0.4);
    ROS_INFO("PID_Z_INT_MAX: %.2f", pid_z_int_max_);

    // Yaw角PID
    nh.param("apoc_pkg/PID_YAW_KP", pid_yaw_kp_, 2.0);
    ROS_INFO("PID_YAW_KP: %.2f", pid_yaw_kp_);
    nh.param("apoc_pkg/PID_YAW_KI", pid_yaw_ki_, 0.05);
    ROS_INFO("PID_YAW_KI: %.2f", pid_yaw_ki_);
    nh.param("apoc_pkg/PID_YAW_KD", pid_yaw_kd_, 0.02);
    ROS_INFO("PID_YAW_KD: %.2f", pid_yaw_kd_);
    nh.param("apoc_pkg/PID_YAW_OUT_MIN", pid_yaw_out_min_, -0.6);
    ROS_INFO("PID_YAW_OUT_MIN: %.2f", pid_yaw_out_min_);
    nh.param("apoc_pkg/PID_YAW_OUT_MAX", pid_yaw_out_max_, 0.6);
    ROS_INFO("PID_YAW_OUT_MAX: %.2f", pid_yaw_out_max_);
    nh.param("apoc_pkg/PID_YAW_INT_MIN", pid_yaw_int_min_, -0.3);
    ROS_INFO("PID_YAW_INT_MIN: %.2f", pid_yaw_int_min_);
    nh.param("apoc_pkg/PID_YAW_INT_MAX", pid_yaw_int_max_, 0.3);
    ROS_INFO("PID_YAW_INT_MAX: %.2f", pid_yaw_int_max_);

    // PID控制频率 & 飞行超时
    nh.param("apoc_pkg/PID_CONTROL_RATE", pid_control_rate_, 50.0);
    ROS_INFO("PID_CONTROL_RATE: %.1f Hz", pid_control_rate_);
    nh.param("apoc_pkg/PID_FLIGHT_TIMEOUT", pid_flight_timeout_, 60.0);
    ROS_INFO("PID_FLIGHT_TIMEOUT: %.1f s", pid_flight_timeout_);
    
    //
    nh.param("apoc_pkg/landing_tolerance", landing_tolerance_, 0.1);
    ROS_INFO("landing_tolerance: %.1f m",landing_tolerance_);
    nh.param("apoc_pkg/landing_timeout", landing_timeout_, 3.0);
    ROS_INFO("landing_timeout: %.1f s", landing_timeout_);
     

    //ros话题初始化
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &apoc::state_cb, this);
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &apoc::local_pos_cb, this);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    
    //
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
    //
    local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

    //变量初始化
    current_state.connected = false;
    current_state.armed = false;//disarm
    current_state.mode = "STABILIZED";  // 默认模式

    //pose
    pose.header.frame_id = "map";
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    //home_pose
    home_pose.header.frame_id = "map";
    home_pose.pose.position.x = 0;
    home_pose.pose.position.y = 0;
    home_pose.pose.position.z = 0;
    home_pose.pose.orientation.x = 0.0;
    home_pose.pose.orientation.y = 0.0;
    home_pose.pose.orientation.z = 0.0;
    home_pose.pose.orientation.w = 1.0;

    //current_pose
    current_pose.header.frame_id = "map";
    current_pose.pose.position.x = 0;
    current_pose.pose.position.y = 0;
    current_pose.pose.position.z = 0;
    current_pose.pose.orientation.x = 0.0;
    current_pose.pose.orientation.y = 0.0;
    current_pose.pose.orientation.z = 0.0;
    current_pose.pose.orientation.w = 1.0;

    last_request = ros::Time::now();

}
