/*
* file:     pid_flab.cpp
* name：    flytoPIDcorrect
* describe：使用PID控制飞行到绝对坐标fly_ab_x/y/z/yaw
* input：   fly_pid_x/y/z/yaw
* output:   true    ->  到达
*           false   ->  未到达
* param:    NONE
* depend:   ros-noetic,cpp,mavros,px4,ununtu20.04,apoc.h
* function: reachCheck，flytoAbsolute
* vision:   1.0
* method：  使用pid控制器计算步长，使用flytoAbsolute飞到中间步，直到到达目的地，reachCheck检查到达
*/
/*
#include "apoc_pkg/apoc.h"
#include "apoc_pkg/pidctrl.h"



bool apoc::flytoPIDcorrect(float fly_pid_x, float fly_pid_y, float fly_pid_z, float fly_pid_yaw) {

    if (!current_state.armed) {
        ROS_WARN("Cannot execute PID flight: Vehicle is not armed");
        return false;
    }

    // 2. 初始化PID控制器（使用带参数的构造函数）
    pidctrl pid_x(
    	pid_x_kp_,pid_x_ki_,pid_x_kd_,
	pid_x_out_min_,pid_x_out_max_,
	pid_x_int_min_,pid_x_int_max_
    );
    pidctrl pid_y(
    	pid_y_kp_,pid_y_ki_,pid_y_kd_,
	pid_y_out_min_,pid_y_out_max_,
	pid_y_int_min_,pid_y_int_max_
    );
    pidctrl pid_z(
    	pid_z_kp_,pid_z_ki_,pid_z_kd_,
	pid_z_out_min_,pid_z_out_max_,
	pid_z_int_min_,pid_z_int_max_
    );
    pidctrl pid_yaw(
    	pid_yaw_kp_,pid_yaw_ki_,pid_yaw_kd_,
	pid_yaw_out_min_,pid_yaw_out_max_,
	pid_yaw_int_min_,pid_yaw_int_max_
    );

    // 3. 设置PID目标值（绝对坐标）
    pid_x.setSetpoint(fly_pid_x);
    pid_y.setSetpoint(fly_pid_y);
    pid_z.setSetpoint(fly_pid_z);

    //待修改

    // 处理偏航角周期性（确保目标角在[-π, π]范围内）
    float normalized_yaw = fmod(fly_pid_yaw, 2 * M_PI);
    if (normalized_yaw > M_PI) normalized_yaw -= 2 * M_PI;
    else if (normalized_yaw < -M_PI) normalized_yaw += 2 * M_PI;
    pid_yaw.setSetpoint(normalized_yaw);

    // 4. 初始化控制变量
    ros::Rate control_rate(pid_control_rate_); // PID控制频率
    ros::Time start_time = ros::Time::now();
    bool target_reached = false;

    ROS_INFO_STREAM("Start PID flight to target: [X:" << fly_pid_x 
                  << ", Y:" << fly_pid_y << ", Z:" << fly_pid_z 
                  << ", Yaw:" << normalized_yaw << " rad]");

    // 5. PID控制主循环（分步飞行直到到达目标）
    while (ros::ok()) {
        // 5.1 检查超时
        if ((ros::Time::now() - start_time).toSec() > pid_flight_timeout_ ) {
            ROS_WARN("PID flight timed out (exceed %ds)", (int)pid_flight_timeout_);
            return false;
        }

        // 5.2 检查是否已到达目标（调用reachCheck函数验证）
        target_reached = reachCheck(fly_pid_x, fly_pid_y, fly_pid_z, normalized_yaw);
        if (target_reached) {
            ROS_INFO("Successfully reached target via PID control");
            return true;
        }

        // 5.3 获取当前位置和姿态
        float current_x = current_pose.pose.position.x;
        float current_y = current_pose.pose.position.y;
        float current_z = current_pose.pose.position.z;

        // 5.4 处理当前偏航角（转换为[-π, π]范围，与目标角统一）
        tf2::Quaternion current_quat(
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w
        );
        tf2::Matrix3x3 current_mat(current_quat);
        double roll, pitch, current_yaw;
        current_mat.getRPY(roll, pitch, current_yaw);
        // 归一化当前偏航角
        current_yaw = fmod(current_yaw, 2 * M_PI);
        if (current_yaw > M_PI) current_yaw -= 2 * M_PI;
        else if (current_yaw < -M_PI) current_yaw += 2 * M_PI;

        // 5.5 计算PID控制输出（步长增量）
        float delta_x = pid_x.compute(current_x);    // X轴步长增量
        float delta_y = pid_y.compute(current_y);    // Y轴步长增量
        float delta_z = pid_z.compute(current_z);    // Z轴步长增量
        float delta_yaw = pid_yaw.compute(current_yaw); // 偏航角步长增量

        // 5.6 计算中间目标点（当前位置 + PID步长增量）
        float via_x = current_x + delta_x;
        float via_y = current_y + delta_y;
        float via_z = current_z + delta_z;
        float via_yaw = current_yaw + delta_yaw;
        // 归一化中间目标偏航角
        via_yaw = fmod(via_yaw, 2 * M_PI);
        if (via_yaw > M_PI) via_yaw -= 2 * M_PI;
        else if (via_yaw < -M_PI) via_yaw += 2 * M_PI;

        // 5.7 调用绝对飞行函数，飞往中间目标点
        // 创建位置设定点消息
        geometry_msgs::PoseStamped setpoint;
        setpoint.header.stamp = ros::Time::now();
        setpoint.header.frame_id = "map"; // 使用地图坐标系

        // 设置位置
        setpoint.pose.position.x = via_x;
        setpoint.pose.position.y = via_y;
        setpoint.pose.position.z = via_z;

        // 将偏航角转换为四元数
        tf2::Quaternion q;
        q.setRPY(0, 0, via_yaw); // 滚转和俯仰设为0，仅控制偏航
        q.normalize();
        setpoint.pose.orientation.x = q.x();
        setpoint.pose.orientation.y = q.y();
        setpoint.pose.orientation.z = q.z();
        setpoint.pose.orientation.w = q.w();

        // 发布位置设定点
        local_pos_pub.publish(setpoint);


        // 5.8 打印调试信息（可选，便于监控控制过程）
        ROS_DEBUG_STREAM("PID Control: Current[X:" << current_x << ", Y:" << current_y 
                       << ", Z:" << current_z << ", Yaw:" << current_yaw 
                       << "] | Delta[X:" << delta_x << ", Y:" << delta_y 
                       << ", Z:" << delta_z << ", Yaw:" << delta_yaw 
                       << "] | Via[X:" << via_x << ", Y:" << via_y 
                       << ", Z:" << via_z << ", Yaw:" << via_yaw << "]");

        // 5.9 控制频率同步
        ros::spinOnce();
        control_rate.sleep();
    }

    // 6. 异常退出处理
    ROS_ERROR("PID flight exited abnormally (ROS node shutdown)");
    return false;
}
*/

/*
* file:     pid_flab.cpp
* name：    flytoPIDcorrect
* describe：使用PID控制飞行到绝对坐标fly_ab_x/y/z/yaw（速度型PID）
* input：   fly_pid_x/y/z/yaw
* output:   true    ->  到达
*           false   ->  未到达
* param:    NONE
* depend:   ros-noetic,cpp,mavros,px4,ununtu20.04,apoc.h
* function: reachCheck，flytoAbsolute
* vision:   1.0
* method：  使用速度型PID计算目标速度，发送速度指令控制飞行，reachCheck检查到达
*/

#include "apoc_pkg/apoc.h"
#include "apoc_pkg/pidctrl.h"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TwistStamped.h> // 速度指令消息头文件

bool apoc::flytoPIDcorrect(float fly_pid_x, float fly_pid_y, float fly_pid_z, float fly_pid_yaw) {
    // 1. 基础状态检查（武装状态、OFFBOARD模式）
    if (!current_state.armed) {
        ROS_WARN("Cannot execute PID flight: Vehicle is not armed");
        return false;
    }

    // 2. 初始化速度型PID控制器（输出限幅为速度最大值/最小值，而非位置增量）
    // 注：PID参数需重新整定（速度型与位置型参数完全不同，例如X轴速度限幅可设±1.5m/s）
    pidctrl pid_x(
        pid_x_kp_, pid_x_ki_, pid_x_kd_,  // 速度PID的Kp/Ki/Kd（需重新整定）
        -pid_x_out_max_, pid_x_out_max_,  // 输出限幅：X轴最大/最小速度（如±1.5）
        -pid_x_int_max_, pid_x_int_max_   // 积分限幅：防止积分饱和
    );
    pidctrl pid_y(
        pid_y_kp_, pid_y_ki_, pid_y_kd_,
        -pid_y_out_max_, pid_y_out_max_,
        -pid_y_int_max_, pid_y_int_max_
    );
    pidctrl pid_z(
        pid_z_kp_, pid_z_ki_, pid_z_kd_,
        -pid_z_out_max_, pid_z_out_max_,  // Z轴速度建议保守（如±0.8m/s）
        -pid_z_int_max_, pid_z_int_max_
    );
    pidctrl pid_yaw(
        pid_yaw_kp_, pid_yaw_ki_, pid_yaw_kd_,
        -pid_yaw_out_max_, pid_yaw_out_max_,  // 偏航角速度限幅（如±0.5rad/s）
        -pid_yaw_int_max_, pid_yaw_int_max_
    );

    // 3. 设置PID目标值（位置目标→用于计算位置误差，进而输出速度）
    pid_x.setSetpoint(fly_pid_x);    // X轴目标位置
    pid_y.setSetpoint(fly_pid_y);    // Y轴目标位置
    pid_z.setSetpoint(fly_pid_z);    // Z轴目标位置
    // 处理偏航角周期性（归一化到[-π, π]，与PX4姿态定义一致）
    float normalized_yaw_target = fmod(fly_pid_yaw, 2 * M_PI);
    normalized_yaw_target = (normalized_yaw_target > M_PI) ? (normalized_yaw_target - 2 * M_PI) : normalized_yaw_target;
    normalized_yaw_target = (normalized_yaw_target < -M_PI) ? (normalized_yaw_target + 2 * M_PI) : normalized_yaw_target;
    pid_yaw.setSetpoint(normalized_yaw_target);  // 偏航角目标位置

    // 4. 初始化控制变量
    ros::Rate control_rate(pid_control_rate_);  // PID控制频率（建议20-50Hz，与速度环匹配）
    ros::Time start_time = ros::Time::now();
    bool target_reached = false;
    // 记录上一时刻时间（用于PID计算dt，确保时间差准确）
    ros::Time last_control_time = ros::Time::now();

    ROS_INFO_STREAM("Start speed-based PID flight to target: "
                  << "[X:" << fly_pid_x << ", Y:" << fly_pid_y << ", Z:" << fly_pid_z 
                  << ", Yaw:" << normalized_yaw_target << " rad]");

    // 5. 速度型PID控制主循环
    while (ros::ok()) {
        // 5.1 超时检查（防止无限循环）
        if ((ros::Time::now() - start_time).toSec() > pid_flight_timeout_) {
            ROS_WARN("PID flight timed out (exceed %ds)", (int)pid_flight_timeout_);
            // 超时后发送零速度，防止无人机失控
            publishZeroVelocity();
            return false;
        }

        // 5.2 目标到达检查（位置误差小于阈值则判定到达）
        target_reached = reachCheck(fly_pid_x, fly_pid_y, fly_pid_z, normalized_yaw_target);
        if (target_reached) {
            ROS_INFO("Successfully reached target via speed PID control");
            // 到达后发送零速度，稳定悬停
            publishZeroVelocity();
            return true;
        }

        // 5.3 获取当前状态（位置、姿态、时间）
        float current_x = current_pose.pose.position.x;
        float current_y = current_pose.pose.position.y;
        float current_z = current_pose.pose.position.z;
        ros::Time current_time = ros::Time::now();

        // 5.4 处理当前偏航角（归一化到[-π, π]）
        tf2::Quaternion current_quat(
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w
        );
        tf2::Matrix3x3 current_mat(current_quat);
        double roll, pitch, current_yaw;
        current_mat.getRPY(roll, pitch, current_yaw);
        current_yaw = fmod(current_yaw, 2 * M_PI);
        current_yaw = (current_yaw > M_PI) ? (current_yaw - 2 * M_PI) : current_yaw;
        current_yaw = (current_yaw < -M_PI) ? (current_yaw + 2 * M_PI) : current_yaw;

        // 5.5 速度型PID计算（核心修改：输入当前位置→输出目标速度）
        // 计算时间差dt（确保PID积分/微分项准确）
        float dt = (current_time - last_control_time).toSec();
        if (dt <= 0.0f || dt > 0.1f) {  // 防止dt异常（如小于0或大于100ms）
            dt = 0.02f;  // 默认dt=20ms（对应50Hz频率）
            ROS_WARN_THROTTLE(1, "Abnormal control interval, use default dt=0.02s");
        }

        // PID计算目标速度（输入当前位置+当前时间，输出目标速度）
        float target_vel_x = pid_x.compute(current_x, current_time);  // X轴目标速度
        float target_vel_y = pid_y.compute(current_y, current_time);  // Y轴目标速度
        float target_vel_z = pid_z.compute(current_z, current_time);  // Z轴目标速度
        float target_ang_vel_yaw = pid_yaw.compute(current_yaw, current_time);  // 偏航目标角速度

        // 5.6 发送速度指令（核心修改：用TwistStamped替代PoseStamped）
        geometry_msgs::TwistStamped vel_cmd;
        vel_cmd.header.stamp = current_time;
        vel_cmd.header.frame_id = "map";  // 速度参考系（map系：全局速度；base_link系：机体速度）
        
        // 线速度（X/Y/Z轴，对应map系全局速度）
        vel_cmd.twist.linear.x = target_vel_x;
        vel_cmd.twist.linear.y = target_vel_y;
        vel_cmd.twist.linear.z = target_vel_z;
        
        // 角速度（仅偏航角控制，滚转/俯仰角速度设为0，防止姿态失控）
        vel_cmd.twist.angular.x = 0.0f;    // 滚转角速度=0
        vel_cmd.twist.angular.y = 0.0f;    // 俯仰角速度=0
        vel_cmd.twist.angular.z = target_ang_vel_yaw;  // 偏航角速度（PID输出）

        // 发布速度指令（需确保local_vel_pub已在apoc类中初始化）
        local_vel_pub.publish(vel_cmd);

        // 5.7 调试信息打印（每1秒打印一次，避免日志刷屏）
        ROS_DEBUG_STREAM_THROTTLE(1, 
            "Speed PID Control: "
            "Current[X:" << current_x << ", Y:" << current_y << ", Z:" << current_z << ", Yaw:" << current_yaw << "] "
            "TargetVel[X:" << target_vel_x << ", Y:" << target_vel_y << ", Z:" << target_vel_z << ", YawVel:" << target_ang_vel_yaw << "] "
            "Error[X:" << (fly_pid_x - current_x) << ", Y:" << (fly_pid_y - current_y) << ", Z:" << (fly_pid_z - current_z) << "]"
        );

        // 5.8 状态更新与频率同步
        last_control_time = current_time;  // 更新上一控制时刻
        ros::spinOnce();                   // 处理ROS回调（如位置订阅）
        control_rate.sleep();              // 确保控制频率稳定
    }

    // 6. 异常退出处理（如ROS节点关闭）
    ROS_ERROR("PID flight exited abnormally (ROS node shutdown)");
    publishZeroVelocity();  // 发送零速度，确保无人机安全
    return false;
}

// 辅助函数：发送零速度指令（防止失控，需在apoc类中声明）
void apoc::publishZeroVelocity() {
    geometry_msgs::TwistStamped zero_vel;
    zero_vel.header.stamp = ros::Time::now();
    zero_vel.header.frame_id = "map";
    // 所有线速度和角速度设为0
    zero_vel.twist.linear.x = 0.0f;
    zero_vel.twist.linear.y = 0.0f;
    zero_vel.twist.linear.z = 0.0f;
    zero_vel.twist.angular.x = 0.0f;
    zero_vel.twist.angular.y = 0.0f;
    zero_vel.twist.angular.z = 0.0f;
    local_vel_pub.publish(zero_vel);
    ROS_INFO("Published zero velocity command");
}
