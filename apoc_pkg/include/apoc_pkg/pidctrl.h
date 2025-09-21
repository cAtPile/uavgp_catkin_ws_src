#ifndef PIDCTRL_H
#define PIDCTRL_H

#include <ros/ros.h>//ros
#include <geometry_msgs/PoseStamped.h>//mav定点
#include <mavros_msgs/CommandBool.h>//arm
#include <mavros_msgs/SetMode.h>//mode
#include <mavros_msgs/State.h>//状态
#include <tf2/LinearMath/Quaternion.h>//四元数
#include <tf2/LinearMath/Matrix3x3.h>//rpy

#include <vector>
#include <cmath>

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