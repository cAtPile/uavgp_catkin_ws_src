
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
#include <geometry_msgs/PoseStamped.h>//mav定点
#include <mavros_msgs/CommandBool.h>//arm
#include <mavros_msgs/SetMode.h>//mode
#include <mavros_msgs/State.h>//状态
#include <tf2/LinearMath/Quaternion.h>//四元数
#include <tf2/LinearMath/Matrix3x3.h>//rpy

#include <vector>
#include <cmath>

//#include <geometry_msgs/TwistStamped.h>//mav速度

class apoc{
private:

    double connect_timeout_;
    double modeswitch_timeout_;
    double armswitch_timeout_;
    double fly_ab_timeout_;
    double reach_tolerance_distance_;
    double reach_tolerance_angle_;
    
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
    
    double landing_timeout_;
    double landing_tolerance_;
    
    ros::NodeHandle nh_;

    //ros句柄
    ros::NodeHandle nh;

    //声明订阅
    ros::Publisher local_pos_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::Subscriber state_sub;
    ros::Subscriber local_pos_sub;

    //初始化消息
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped home_pose;//home位置
    geometry_msgs::PoseStamped current_pose;//当前位置
    ros::Time last_request;
    ros::Rate rate;

    //回调函数
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

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
    bool flytoPIDcorrect
        (float fly_pid_x , float fly_pid_y , float fly_pid_z , float fly_pid_yaw );//pid修正
    bool flytoRelative
        (float fly_re_x , float fly_re_y , float fly_re_z , float fly_re_yaw );//相对修正
    bool hoverSwitch(float hover_time);//悬停
    bool landSwitch();//降落
    //待定track.cpp追踪
    //bool flytoRelative()

};

class pidctrl{
private:
    // PID参数
    double kp;          // 比例系数
    double ki;          // 积分系数
    double kd;          // 微分系数
    
    // 限制输出范围
    double output_min;  // 输出最小值
    double output_max;  // 输出最大值
    
    // 积分项限制，防止积分饱和
    double integral_min; // 积分最小值
    double integral_max; // 积分最大值
    
    // PID计算变量
    double setpoint;    // 目标值
    double last_error;  // 上一次误差
    double integral;    // 积分项
    double derivative;  // 微分项
    double output;      // 输出值
    
    // 时间变量
    ros::Time last_time; // 上一次计算时间
    bool is_first_run;   // 是否首次运行
public:
    
    pidctrl();// 构造函数
    pidctrl(double kp,double ki,double kd, 
            double out_min,double out_max,
            double int_min,double int_max );
    // 设置PID参数
    void setPIDctrlParams(  
        double k_p, double k_i, double k_d,  
        double int_min, double int_max, 
        double out_min, double out_max     );
    void setSetpoint(float sp);// 设置目标值
    float getSetpoint();
    float getOutput();// 获取当前输出值
    void reset();// 重置PID控制器
    float compute(float process_value, ros::Time current_time);
    float compute(float process_value);// 计算PID输出 (自动使用当前时间)

};

#endif
