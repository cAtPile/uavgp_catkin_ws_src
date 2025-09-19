#include "apoc_pkg/apoc.h"
#include "apoc_pkg/pidctrl.h"

void apoc::trackSwitch() {
    
    //
    geometry_msgs::PoseStamped trace_pose;
    trace_pose.header.frame_id = "map"; 
    trace_pose.pose.position.z = 1;
    trace_pose.pose.orientation.x = 0.0;
    trace_pose.pose.orientation.y = 0.0;
    trace_pose.pose.orientation.z = 0.0;
    trace_pose.pose.orientation.w = 1.0;

    // 计算校正系数
    float correct_ratio = current_pose.pose.position.z * trace_cam_ratio_;
    
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
        if (fabs(current_detection.detection_x - trace_target_center_x_) <= trace_tolerance_ &&
            fabs(current_detection.detection_y - trace_target_center_y_) <= trace_tolerance_) {
            ROS_INFO("Target reached within tolerance");
            break;
        }
        
        // 1. 坐标转换：将检测到的目标中心转换为控制量
        float target_offset_x = (current_detection.detection_x - trace_target_center_x_) * correct_ratio + current_pose.pose.position.x ;
        float target_offset_y = (current_detection.detection_y - trace_target_center_y_) * correct_ratio + current_pose.pose.position.y ;

        // 8. PID控制：设置目标偏差，计算控制量（输入=当前实际偏差）
        pid_x.setSetpoint(target_offset_x);  // PID期望偏差=目标偏差
        pid_y.setSetpoint(target_offset_y);
        float delta_x = pid_x.compute(current_pose.pose.position.x);  // 计算X轴增量
        float delta_y = pid_y.compute(current_pose.pose.position.y);  // 计算Y轴增量

        // 计算下一步位置
        float via_x = current_pose.pose.position.x + delta_x;
        float via_y = current_pose.pose.position.y + delta_y;

        // 发送飞行指令
        trace_pose.pose.position.x = via_x;
        trace_pose.pose.position.y = via_y;

        //发布定点
        local_pos_pub.publish(trace_pose);
        
        // 处理回调并控制循环速率
        ros::spinOnce();
        rate.sleep();
        
        // 超时退出
        if ((ros::Time::now() - start).toSec() > trace_timeout_) {
            ROS_WARN("Tracking timed out");
            break;
        }
    }
}
