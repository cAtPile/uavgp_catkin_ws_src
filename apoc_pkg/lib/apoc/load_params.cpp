//load param
#include "apoc_pkg/apoc.h"

void apoc::loadParams() {

    // ============== 超时参数获取==============
    nh.param("apoc_pkg/connect_timeout", connect_timeout_, 10.0);
    nh.param("apoc_pkg/modeswitch_timeout", modeswitch_timeout_, 10.0);
    nh.param("apoc_pkg/armswitch_timeout", armswitch_timeout_, 10.0);
    nh.param("apoc_pkg/fly_ab_timeout", fly_ab_timeout_, 30.0);
    nh.param("apoc_pkg/landing_timeout", landing_timeout_, 10.0);
    nh.param("apoc_pkg/trace_timeout", trace_timeout_, 60.0);
    nh.param("apoc_pkg/PID_FLIGHT_TIMEOUT", pid_flight_timeout_, 60.0);
    
    // ============== 容忍值参数获取==============
    nh.param("apoc_pkg/reach_tolerance_distance", reach_tolerance_distance_, 0.1);
    nh.param("apoc_pkg/reach_tolerance_angle", reach_tolerance_angle_, 0.1);
    nh.param("apoc_pkg/landing_tolerance", landing_tolerance_, 0.1);
    /*
    nh.param("apoc_pkg/trace_tolerance", trace_tolerance_, 20.0);
    在追踪参数部分
    */

    // ============== 追踪参数获取==============
    nh.param("apoc_pkg/trace_cam_ratio", trace_cam_ratio_, 0.005);
    nh.param("apoc_pkg/trace_target_center_x", trace_target_center_x_, 320.0);
    nh.param("apoc_pkg/trace_target_center_y", trace_target_center_y_, 320.0);
    nh.param("apoc_pkg/trace_tolerance", trace_tolerance_, 20.0);
    /*
    nh.param("apoc_pkg/trace_timeout", trace_timeout_, 60.0);
    在超时部分
    */
     
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
    /*
    nh.param("apoc_pkg/PID_FLIGHT_TIMEOUT", pid_flight_timeout_, 60.0);
    在超时参数部分
    */

    ROS_INFO("\n==================== Timeout Params Loaded ====================");、\
    ROS_INFO("connect_timeout: %.1f s", connect_timeout_);
    ROS_INFO("modeswitch_timeout: %.1f s", modeswitch_timeout_);
    ROS_INFO("armswitch_timeout: %.1f s", armswitch_timeout_);
    ROS_INFO("fly_ab_timeout: %.1f s", fly_ab_timeout_);
    ROS_INFO("landing_timeout: %.1f s", landing_timeout_);
    ROS_INFO("trace_timeout: %.1f s", trace_timeout_);
    ROS_INFO("pid_flight_timeout: %.1f s", pid_flight_timeout_);

    ROS_INFO("\n==================== Tolerance Params Loaded ====================");
    ROS_INFO("reach_tolerance_distance: %.1f m", reach_tolerance_distance_);
    ROS_INFO("reach_tolerance_angle: %.1f angle", reach_tolerance_angle_);
    ROS_INFO("landing_tolerance: %.1f m", landing_tolerance_);
    ROS_INFO("trace_tolerance: %.1f pix", trace_tolerance_);

    ROS_INFO("\n==================== Trace Params Loaded ====================");
    ROS_INFO("trace_tolerance: %.1f NONE", trace_cam_ratio_);
    ROS_INFO("trace_tolerance: %.1f pix", trace_target_center_x_);
    ROS_INFO("trace_tolerance: %.1f pix", trace_target_center_y_);
    ROS_INFO("trace_tolerance: %.1f pix", trace_tolerance_);
    ROS_INFO("trace_tolerance: %.1f s", trace_timeout_);


    ROS_INFO("\n==================== PID Params Loaded ====================");
    ROS_INFO("PID_CONTROL_RATE: %.1f Hz", pid_control_rate_);
    ROS_INFO("PID_FLIGHT_TIMEOUT: %.1f s", pid_flight_timeout_);
    
    ROS_INFO("\n PID_X ");
    ROS_INFO("KP: %.2f | KI: %.2f | KD: %.2f", pid_x_kp_, pid_x_ki_, pid_x_kd_);
    ROS_INFO("Output: [%.1f, %.1f] | Int: [%.1f, %.1f]", 
             pid_x_out_min_, pid_x_out_max_, pid_x_int_min_, pid_x_int_max_);
    
    ROS_INFO("\n PID_Y");
    ROS_INFO("KP: %.2f | KI: %.2f | KD: %.2f", pid_y_kp_, pid_y_ki_, pid_y_kd_);
    ROS_INFO("Output: [%.1f, %.1f] | Int: [%.1f, %.1f]", 
             pid_y_out_min_, pid_y_out_max_, pid_y_int_min_, pid_y_int_max_);
    
    ROS_INFO("\n PID_Z");
    ROS_INFO("KP: %.2f | KI: %.2f | KD: %.2f", pid_z_kp_, pid_z_ki_, pid_z_kd_);
    ROS_INFO("Output: [%.1f, %.1f] | Int: [%.1f, %.1f]", 
             pid_z_out_min_, pid_z_out_max_, pid_z_int_min_, pid_z_int_max_);
    
    ROS_INFO("\n PID_Yaw");
    ROS_INFO("KP: %.2f | KI: %.2f | KD: %.2f", pid_yaw_kp_, pid_yaw_ki_, pid_yaw_kd_);
    ROS_INFO("Output: [%.1f, %.1f] | Int: [%.1f, %.1f]", 
             pid_yaw_out_min_, pid_yaw_out_max_, pid_yaw_int_min_, pid_yaw_int_max_);
    
    ROS_INFO("\n==============================================================");
}  