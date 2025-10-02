
/*
* file：    apoc.h
* name:     apoc.h
* discribe: apoc头文件
* vision:   1.0
* depend:   ununtu20.04,ros-noetic
* class:    apoc
*           pidctrl
* path:
*/

#ifndef APOC_H
#define APOC_H

#include <ros/ros.h>//ros
#include <geometry_msgs/PoseStamped.h>//定点
#include <mavros_msgs/CommandBool.h>//arm
#include <mavros_msgs/SetMode.h>//mode
#include <mavros_msgs/State.h>//状态
#include <mavros_msgs/Trajectory.h>
#include <tf2/LinearMath/Quaternion.h>//四元数
#include <tf2/LinearMath/Matrix3x3.h>//rpy
#include <geometry_msgs/TwistStamped.h>//mav速度

#include <sensor_msgs/LaserScan.h>

#include <apoc_pkg/detection_data.h>//检测

#include <vector>
#include <cmath>

#include <std_msgs/Bool.h>

#include <mutex>

class apoc{
private:

    //------------飞行参量----------------------
    double connect_timeout_;
    double modeswitch_timeout_;
    double armswitch_timeout_;
    double fly_ab_timeout_;
    double reach_tolerance_distance_;
    double reach_tolerance_angle_;
    double landing_timeout_;
    double landing_tolerance_;
    double trace_cam_ratio_;
    double trace_target_center_x_;
    double trace_target_center_y_;
    double trace_tolerance_;
    double trace_timeout_;
    double TRACE_CAM_RATIO;
    double TRACE_TARGET_CENTER_X;
    double TRACE_TARGET_CENTER_Y;
    double TRACE_TOLERANCE;
    double TRACE_TIMEOUT;
    
    // -------------- PID参数变量--------------
    // X轴PID
    double pid_x_kp_;
    double pid_x_ki_;
    double pid_x_kd_;
    double pid_x_out_min_;
    double pid_x_out_max_;
    double pid_x_int_min_;
    double pid_x_int_max_;

    // Y轴PID
    double pid_y_kp_;
    double pid_y_ki_;
    double pid_y_kd_;
    double pid_y_out_min_;
    double pid_y_out_max_;
    double pid_y_int_min_;
    double pid_y_int_max_;

    // Z轴PID
    double pid_z_kp_;
    double pid_z_ki_;
    double pid_z_kd_;
    double pid_z_out_min_;
    double pid_z_out_max_;
    double pid_z_int_min_;
    double pid_z_int_max_;

    // Yaw角PID
    double pid_yaw_kp_;
    double pid_yaw_ki_;
    double pid_yaw_kd_;
    double pid_yaw_out_min_;
    double pid_yaw_out_max_;
    double pid_yaw_int_min_;
    double pid_yaw_int_max_;

    // PID控制频率 & 飞行超时
    double pid_control_rate_;       // 控制频率(Hz)
    double pid_flight_timeout_;  // 飞行超时(s)

    //ros句柄
    ros::NodeHandle nh;

    /*声明订阅*/
    ros::Publisher local_pos_pub;
    ros::Publisher local_vel_pub;  // 速度指令发布器
    ros::Publisher detection_action_pub;//识别使能
    ros::Publisher goal_pub;

    ros::Publisher mav_trajectory_sub;
    ros::Publisher mav_obstacle_sub;

    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::Subscriber state_sub;
    ros::Subscriber local_pos_sub;
    ros::Subscriber detection_data_sub; //订阅识别
    ros::Subscriber lp_setpose_sub;
    ros::Subscriber lp_setvel_sub;
    ros::Subscriber lp_trajectory_sub;
    ros::Subscriber lp_obstacle_sub;

    //初始化消息
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped home_pose;//home位置
    geometry_msgs::PoseStamped current_pose;//当前位置

    // 存储当前检测到的目标信息
    apoc_pkg::detection_data current_detection;
    std_msgs::Bool detection_action;
    geometry_msgs::Twist current_lp_sv;
    geometry_msgs::PoseStamped current_lp_sp;
    mavros_msgs::Trajectory current_lp_trajectory;
    sensor_msgs::LaserScan current_lp_obstacle;

    ros::Time last_request;
    ros::Rate rate;

    //线程互斥锁
    mutable std::mutex current_pose_mutex_;
    mutable std::mutex home_pose_mutex_;           // 新增Home位置互斥锁

    //回调函数
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void detection_data_cb(const apoc_pkg::detection_data::ConstPtr& msg);

    //
    void loadParams();

public:

    apoc();//构造函数
    bool connectSwitch();//连接
    bool modeSwitch(int mode_key);//mode
    bool armSwitch(int arm_key);//解锁
    bool takeoffSwitch(float takeoff_alt);//起飞
    bool reachCheck
        (float check_x , float check_y , float check_z , float check_yaw );//到达检查
    bool flytoAbsolute
        (float fly_ab_x , float fly_ab_y , float fly_ab_z , float fly_ab_yaw ); //绝对飞行
    bool flytoRelative
        (float fly_re_x , float fly_re_y , float fly_re_z , float fly_re_yaw );//相对修正
    bool hoverSwitch(float hover_time);//悬停
    void landSwitch();//降落
    void publishZeroVelocity();//0速度发布
    void trackSwitch();
    //void plannerSwitch();

};

#endif
