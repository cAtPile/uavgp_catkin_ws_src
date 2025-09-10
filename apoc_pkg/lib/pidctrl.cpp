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
pidctrl::pidctrl(   float k_p, float k_i, float k_d, 
                    float out_min, float out_max, 
                    float int_min, float int_max) {
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

    // 设置PID参数
void pidctrl::setPIDctrlParams(  float k_p, float k_i, float k_d,  
                            float out_min, float out_max , 
                            float int_min, float int_max){
    kp = k_p;
    ki = k_i;
    kd = k_d;
    
    output_min = out_min;
    output_max = out_max;
    
    integral_min = int_min;
    integral_max = int_max;

}

// 设置目标值
void setSetpoint(float sp){
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
