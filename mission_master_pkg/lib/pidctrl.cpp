#include "PIDController.h"

// 构造函数，带输出限幅、积分限幅参数
PIDController::PIDController(double Kp, double Ki, double Kd,
                             double output_min, double output_max,
                             double integral_min, double integral_max)
    : Kp_(Kp), Ki_(Ki), Kd_(Kd),
      output_min_(output_min), output_max_(output_max),
      integral_min_(integral_min), integral_max_(integral_max),
      integral_(0.0), prev_error_(0.0)
{}

// 设置 PID 参数
void PIDController::setPID(double Kp, double Ki, double Kd)
{
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
}

// 设置输出限幅
void PIDController::setOutputLimit(double min_output, double max_output)
{
    output_min_ = min_output;
    output_max_ = max_output;
}

// 设置积分限幅
void PIDController::setIntegralLimit(double min_integral, double max_integral)
{
    integral_min_ = min_integral;
    integral_max_ = max_integral;
}

// 重置 PID 内部状态
void PIDController::reset()
{
    integral_ = 0.0;
    prev_error_ = 0.0;
}

// 计算 PID 输出
double PIDController::compute(double error, double dt)
{
    if (dt <= 0) return 0;

    // ---- P 项 ----
    double Pout = Kp_ * error;

    // ---- I 项（积分）----
    integral_ += error * dt;

    // 限制积分项，防止积分饱和
    if (integral_ > integral_max_) integral_ = integral_max_;
    if (integral_ < integral_min_) integral_ = integral_min_;

    double Iout = Ki_ * integral_;

    // ---- D 项（微分）----
    double derivative = (error - prev_error_) / dt;
    double Dout = Kd_ * derivative;

    prev_error_ = error;

    // ---- PID 输出 ----
    double output = Pout + Iout + Dout;

    // ---- 输出限幅（关键）----
    if (output > output_max_) output = output_max_;
    if (output < output_min_) output = output_min_;

    return output;
}
