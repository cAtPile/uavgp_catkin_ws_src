/*
trace_data.msg
uint16 trace_x;   // 目标在图像中的X坐标
uint16 trace_y;   // 目标在图像中的Y坐标
uint8 trace_id;   // 目标ID，用于多目标跟踪*/

void apoc::trace_data_cb(const/*trace_data*/){
    current_trace = *msg;
}
#include "apoc_pkg/apoc.h"
void apoc::trackSwitch(){
    float correct_ration = current_pose.pose.position.z*CAM_RATION;
    //PID
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
    pid_y.reset();
    pid_x.reset();

    ros::Time start = ros::Time::now();

    while (ros::ok()){
        //达到阈值退出
        if (current_trace.trace_x <= TRACE_TOLERANCE &&
            current_trace.trace_ <= TRACE_TOLERANCE 
            ){
                break;
            }

        //1.换算
        float local_trace_x=current_trace.trace_x * correct_ration ;
        float local_trace_y=current_trace.trace_y * correct_ration ;

        pid_x.setSetpoint(local_trace_x);
        pid_y.setSetpoint(local_trace_y);

        float delta_x = pid_x.compute(current_x);    // X轴步长增量
        float delta_y = pid_y.compute(current_y);    // Y轴步长增量

        float via_x = current_pose.pose.position.x + delta_x;
        float via_y = current_pose.pose.position.y + delta_y;

        flytoRelative(via_x,via_y,SET_ALTITUDE,SET_ORIENTATION);

        ros::spinOnce();
        control_rate.sleep();
        
        //超时退出
        if ((ros::Time::now() - start).toSec() >TRACE_TIMEOUT)
        {
            break;
        }
        

    }

}

