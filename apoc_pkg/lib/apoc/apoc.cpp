
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

    // ============== 参数获取==============
    nh.param("apoc_pkg/connect_timeout", connect_timeout_, 10.0);
    nh.param("apoc_pkg/modeswitch_timeout", modeswitch_timeout_, 10.0);
    nh.param("apoc_pkg/armswitch_timeout", armswitch_timeout_, 10.0);
    nh.param("apoc_pkg/fly_ab_timeout", fly_ab_timeout_, 30.0);
    nh.param("apoc_pkg/reach_tolerance_distance", reach_tolerance_distance_, 0.1);
    nh.param("apoc_pkg/reach_tolerance_angle", reach_tolerance_angle_, 0.1);
    nh.param("apoc_pkg/landing_tolerance", landing_tolerance_, 0.1);
    nh.param("apoc_pkg/landing_timeout", landing_timeout_, 3.0);
    nh.param("apoc_pkg/trace_cam_ratio", trace_cam_ratio_, 0.005);
    nh.param("apoc_pkg/trace_target_center_x", trace_target_center_x_, 320.0);
    nh.param("apoc_pkg/trace_target_center_y", trace_target_center_y_, 320.0);
    nh.param("apoc_pkg/trace_tolerance", trace_tolerance_, 20.0);
    nh.param("apoc_pkg/trace_timeout", trace_timeout_, 60.0);
     
    // ============== PID参数获取==============
    // X轴PID
    nh.param("apoc_pkg/PID_X_KP", pid_x_kp_, 2.5);
    nh.param("apoc_pkg/PID_X_KI", pid_x_ki_, 0.1);
    nh.param("apoc_pkg/PID_X_KD", pid_x_kd_, 0.05);
    nh.param("apoc_pkg/PID_X_OUT_MIN", pid_x_out_min_, -1.0);
    nh.param("apoc_pkg/PID_X_OUT_MAX", pid_x_out_max_, 1.0);
    nh.param("apoc_pkg/PID_X_INT_MIN", pid_x_int_min_, -0.5);
    nh.param("apoc_pkg/PID_X_INT_MAX", pid_x_int_max_, 0.5);

    // Y轴PID
    nh.param("apoc_pkg/PID_Y_KP", pid_y_kp_, 2.5);
    nh.param("apoc_pkg/PID_Y_KI", pid_y_ki_, 0.1);
    nh.param("apoc_pkg/PID_Y_KD", pid_y_kd_, 0.05);
    nh.param("apoc_pkg/PID_Y_OUT_MIN", pid_y_out_min_, -1.0);
    nh.param("apoc_pkg/PID_Y_OUT_MAX", pid_y_out_max_, 1.0);
    nh.param("apoc_pkg/PID_Y_INT_MIN", pid_y_int_min_, -0.5);
    nh.param("apoc_pkg/PID_Y_INT_MAX", pid_y_int_max_, 0.5);

    // Z轴PID
    nh.param("apoc_pkg/PID_Z_KP", pid_z_kp_, 3.0);
    nh.param("apoc_pkg/PID_Z_KI", pid_z_ki_, 0.2);
    nh.param("apoc_pkg/PID_Z_KD", pid_z_kd_, 0.1);
    nh.param("apoc_pkg/PID_Z_OUT_MIN", pid_z_out_min_, -0.8);
    nh.param("apoc_pkg/PID_Z_OUT_MAX", pid_z_out_max_, 0.8);
    nh.param("apoc_pkg/PID_Z_INT_MIN", pid_z_int_min_, -0.4);
    nh.param("apoc_pkg/PID_Z_INT_MAX", pid_z_int_max_, 0.4);

    // Yaw角PID
    nh.param("apoc_pkg/PID_YAW_KP", pid_yaw_kp_, 2.0);
    nh.param("apoc_pkg/PID_YAW_KI", pid_yaw_ki_, 0.05);
    nh.param("apoc_pkg/PID_YAW_KD", pid_yaw_kd_, 0.02);
    nh.param("apoc_pkg/PID_YAW_OUT_MIN", pid_yaw_out_min_, -0.6);
    nh.param("apoc_pkg/PID_YAW_OUT_MAX", pid_yaw_out_max_, 0.6);
    nh.param("apoc_pkg/PID_YAW_INT_MIN", pid_yaw_int_min_, -0.3);
    nh.param("apoc_pkg/PID_YAW_INT_MAX", pid_yaw_int_max_, 0.3);
    
    // PID控制频率 & 飞行超时
    nh.param("apoc_pkg/PID_CONTROL_RATE", pid_control_rate_, 50.0);
    nh.param("apoc_pkg/PID_FLIGHT_TIMEOUT", pid_flight_timeout_, 60.0);
    
    /*****sub订阅初始化*****/
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &apoc::state_cb, this);
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &apoc::local_pos_cb, this);
        //识别话题
    detection_data_sub = nh.subscribe<apoc_pkg::detection_data>("/detection/data",10,&apoc::detection_data_cb, this);
    //lp_setvel_sub = nh.advertise<geometry_msgs::Twist>("apoc/setpoint_velocity/cmd_vel_unstamped", 10, lp_setvel_cb,this);
    //lp_setpose_sub = nh.advertise<geometry_msgs::PoseStamped>("apoc/setpoint_position/local", 10, lp_setpose_cb,this);
    //lp_trajectory_sub = nh.advertise<mavros_msgs::Trajectory>("apoc/trajectory/generated", 10,lp_trajectory_cb,this);
    //lp_obstacle_sub = nh.advertise<sensor_msgs::LaserScan>("apoc/obstacle/send", 10,lp_obstacle_cb,this);

    /*****pub发布初始化*****/
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);  
        //识别使能
    detection_action_pub = nh.advertise<std_msgs::Bool>("/tracker_action", 1);
    //mav_trajectory_pub = nh_.advertise<mavros_msgs::Trajectory>("mavros/trajectory/generated", 10);
    //mav_obstacle_pub = nh_.advertise<sensor_msgs::LaserScan>("mavros/obstacle/send", 10);  
    
    /*****clinet服务初始化*****/    
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //current_state
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

    //current_detection
    current_detection.detection_id = 0 ;
    current_detection.detection_x = 0 ;
    current_detection.detection_y = 0 ;

    //
    detection_action.data = false;

    //
    last_request = ros::Time::now();

}