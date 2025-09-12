#include "apoc_pkg/pidctrl.h"

// 构造函数
pidctrl::pidctrl(){
    
    kp = 0.0f;
    ki = 0.0f;
    kd = 0.0f;
    
    output_min = -INFINITY;
    output_max = INFINITY;
    
    integral_min = -INFINITY;
    integral_max = INFINITY;
    
    setpoint = 0.0f;
    last_error = 0.0f;
    integral = 0.0f;
    derivative = 0.0f;
    output = 0.0f;
    
    is_first_run = true;
}

// 带完整参数的构造函数
pidctrl::pidctrl(   double k_p, double k_i, double k_d, 
                    double out_min, double out_max, 
                    double int_min, double int_max) {
    kp = k_p;
    ki = k_i;
    kd = k_d;
    
    output_min = out_min;
    output_max = out_max;
    
    integral_min = int_min;
    integral_max = int_max;
    
    setpoint = 0.0f;
    last_error = 0.0f;
    integral = 0.0f;
    derivative = 0.0f;
    output = 0.0f;
    
    is_first_run = true;
}



// 设置目标值
void pidctrl::setSetpoint(float sp){
    setpoint = sp;
}

float pidctrl::getSetpoint() {
    return setpoint;
}

// 获取当前输出值
float pidctrl::getOutput() {
    return output;
}
    
// 重置PID控制器
void pidctrl::reset() {
    last_error = 0.0f;
    integral = 0.0f;
    derivative = 0.0f;
    output = 0.0f;
    is_first_run = true;
}

