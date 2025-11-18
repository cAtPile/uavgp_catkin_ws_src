#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController
{
public:
    // 构造函数，带输出限幅、积分限幅参数
    PIDController(double Kp, double Ki, double Kd,
                  double output_min = -1.0, double output_max = 1.0,
                  double integral_min = -1.0, double integral_max = 1.0);

    // 设置 PID 参数
    void setPID(double Kp, double Ki, double Kd);

    // 设置输出限幅
    void setOutputLimit(double min_output, double max_output);

    // 设置积分限幅
    void setIntegralLimit(double min_integral, double max_integral);

    // 重置 PID 内部状态
    void reset();

    // 计算 PID 输出
    double compute(double error, double dt);

private:
    double Kp_, Ki_, Kd_;                  // PID 参数
    double output_min_, output_max_;        // 输出限幅
    double integral_min_, integral_max_;    // 积分项限幅

    double integral_;       // 积分项
    double prev_error_;     // 上一误差，用于计算微分
};

#endif // PIDCONTROLLER_H
