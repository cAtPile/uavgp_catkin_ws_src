#include "apoc_pkg/pidctrl.h"

// 计算PID输出 (带当前时间参数)
float pidctrl::compute(float process_value, ros::Time current_time) {
    // 计算当前误差
    float error = setpoint - process_value;
    float dt = 0.0f;
    
    // 处理首次运行情况
    if (is_first_run) {
        last_time = current_time;
        last_error = error;
        is_first_run = false;
        output = 0.0f;
        return output;
    }
    
    // 计算时间差
    dt = (current_time - last_time).toSec();
    
    // 防止时间差为零或负数导致的问题
    if (dt <= 0.0f) {
        return output;
    }
    
    // 计算积分项并限幅
    integral += ki * error * dt;
    if (integral > integral_max) {
        integral = integral_max;
    } else if (integral < integral_min) {
        integral = integral_min;
    }
    
    // 计算微分项（使用当前误差与上次误差的差值）
    derivative = kd * (error - last_error) / dt;
    
    // 计算PID总输出
    output = kp * error + integral + derivative;
    
    // 输出限幅
    if (output > output_max) {
        output = output_max;
    } else if (output < output_min) {
        output = output_min;
    }
    
    // 保存当前状态用于下次计算
    last_error = error;
    last_time = current_time;
    
    return output;
}

// 计算PID输出 (自动使用当前时间)
float pidctrl::compute(float process_value) {
    return compute(process_value, ros::Time::now());
}
