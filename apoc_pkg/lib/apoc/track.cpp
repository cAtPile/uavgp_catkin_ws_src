#include "apoc_pkg/apoc.h"

void apoc::trackSwitch() {
/*
    // 计算校正系数
    float correct_ratio = current_pose.pose.position.z * CAM_RATIO;
    
    // 初始化PID控制器
    pidctrl pid_x(
        pid_x_kp_, pid_x_ki_, pid_x_kd_,
        -pid_x_out_max_, pid_x_out_max_,
        -pid_x_int_max_, pid_x_int_max_
    );
    pidctrl pid_y(
        pid_y_kp_, pid_y_ki_, pid_y_kd_,
        -pid_y_out_max_, pid_y_out_max_,
        -pid_y_int_max_, pid_y_int_max_
    );
    pid_y.reset();
    pid_x.reset();

    ros::Time start = ros::Time::now();

    while (ros::ok()) {
        // 检查是否有有效的检测目标
        if (current_detection.class_name.empty()) {
            ROS_WARN("No detection target available");
            ros::spinOnce();
            control_rate.sleep();
            continue;
        }

        // 达到位置阈值时退出追踪
        if (fabs(current_detection.center_x - TARGET_CENTER_X) <= TRACE_TOLERANCE &&
            fabs(current_detection.center_y - TARGET_CENTER_Y) <= TRACE_TOLERANCE) {
            ROS_INFO("Target reached within tolerance");
            break;
        }

        // 1. 坐标转换：将检测到的目标中心转换为控制量
        float target_offset_x = (current_detection.center_x - TARGET_CENTER_X) * correct_ratio;
        float target_offset_y = (current_detection.center_y - TARGET_CENTER_Y) * correct_ratio;

        // 设置PID目标值
        pid_x.setSetpoint(target_offset_x);
        pid_y.setSetpoint(target_offset_y);

        // 计算控制量
        float delta_x = pid_x.compute(current_x);    // X轴步长增量
        float delta_y = pid_y.compute(current_y);    // Y轴步长增量

        // 计算下一步位置
        float via_x = current_pose.pose.position.x + delta_x;
        float via_y = current_pose.pose.position.y + delta_y;

        // 发送飞行指令
        flytoRelative(via_x, via_y, SET_ALTITUDE, SET_ORIENTATION);

        // 处理回调并控制循环速率
        ros::spinOnce();
        control_rate.sleep();
        
        // 超时退出
        if ((ros::Time::now() - start).toSec() > TRACE_TIMEOUT) {
            ROS_WARN("Tracking timed out");
            break;
        }
    }
    */
}
