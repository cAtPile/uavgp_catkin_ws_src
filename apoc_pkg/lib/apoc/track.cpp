#include "apoc_pkg/apoc.h"
#include "apoc_pkg/pidctrl.h"

void apoc::trackSwitch() {

    // 1. 初始化参数（修正CAM_RATIO，添加初始位置记录）
    float CAM_RATIO = 0.005f;        // 校准后的像素→米转换系数（5mm/像素）
    float TARGET_CENTER_X = 320.0f;  // 图像中心X（如640x640分辨率）
    float TARGET_CENTER_Y = 320.0f;  // 图像中心Y
    float TRACE_TOLERANCE = 20.0f;   // 追踪容差（20像素）
    float TRACE_TIMEOUT = 60.0f;     // 超时时间（20秒）
    ros::Rate rate(20);              // 循环频率20Hz（50ms/次）
    float init_x = current_pose.pose.position.x;  // 初始追踪X位置（基准）
    float init_y = current_pose.pose.position.y;  // 初始追踪Y位置（基准）

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
        if (current_detection.detection_id == 0) {
            ROS_WARN("No detection target available");
            ros::spinOnce();
            rate.sleep();
            continue;
        }

        // 达到位置阈值时退出追踪
        if (fabs(current_detection.detection_x - TARGET_CENTER_X) <= TRACE_TOLERANCE &&
            fabs(current_detection.detection_y - TARGET_CENTER_Y) <= TRACE_TOLERANCE) {
            ROS_INFO("Target reached within tolerance");
            break;
        }
        
        // 1. 坐标转换：将检测到的目标中心转换为控制量
        float target_offset_x = (current_detection.detection_x - TARGET_CENTER_X) * correct_ratio;
        float target_offset_y = (current_detection.detection_y - TARGET_CENTER_Y) * correct_ratio;

        // 8. PID控制：设置目标偏差，计算控制量（输入=当前实际偏差）
        float current_offset_x = current_pose.pose.position.x - init_x;  // 当前实际X偏差
        float current_offset_y = current_pose.pose.position.y - init_y;  // 当前实际Y偏差
        pid_x.setSetpoint(target_offset_x);  // PID期望偏差=目标偏差
        pid_y.setSetpoint(target_offset_y);
        float delta_x = pid_x.compute(current_offset_x);  // 计算X轴增量
        float delta_y = pid_y.compute(current_offset_y);  // 计算Y轴增量

        // 计算下一步位置
        float via_x = current_pose.pose.position.x + delta_x;
        float via_y = current_pose.pose.position.y + delta_y;

        // 发送飞行指令
        flytoRelative(via_x, via_y, 1, 0);

        // 处理回调并控制循环速率
        ros::spinOnce();
        rate.sleep();
        
        // 超时退出
        if ((ros::Time::now() - start).toSec() > TRACE_TIMEOUT) {
            ROS_WARN("Tracking timed out");
            break;
        }
    }
}
