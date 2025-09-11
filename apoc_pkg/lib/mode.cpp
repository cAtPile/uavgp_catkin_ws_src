
/*
* file：    mode.cpp
* name：    modeSwitch
* path:     /lib
* describe：switch mode
* input：   mode_key = 1        ->  offboard
*           mode_key = 0/其他   ->  自稳
* output:   true    ->  成功切换到目标模式
*           false   ->  未切换到目标模式
* param:    MODESWITCH_TIIMEOUT
* value:    
* depend:   ros-noetic,cpp,mavros,px4,ununtu20.04,apoc.h
* function: NONE
* vision:   1.0
* method:   
* info:     "Not connected to FCU, cannot set mode"
*           未连接无法设置
*
*           "Timeout setting " << target_mode << " mode"
*           setmode超时
*
*           "ROS node is not running properly during setmode"
*           setmode时ros节点失效
*
*           "Modeset failed unkunown reason unexpectedly!"
*           未知原因失效
*/

#include "apoc_pkg/apoc.h"

bool apoc::modeSwitch(int mode_key) {

    // 根据输入参数确定目标模式
    std::string target_mode;
    if (mode_key == 1) {
        target_mode = "OFFBOARD";  // mode_key=1时切换到offboard模式
    } else {
        target_mode = "STABILIZED"; // mode_key=0或其他值时切换到自稳模式
    }

    // 检查是否已经处于目标模式
    if (current_state.mode == target_mode) {
        return true;
    }
        
    // 连接检查
    if (!current_state.connected) {
        ROS_WARN("Not connected to FCU, cannot set mode");
        return false;
    }

    // 超时基准，使用参数定义的超时时间
    ros::Time start = ros::Time::now();

    // 循环尝试切换模式直到成功或超时
    while (ros::ok() && current_state.mode != target_mode) {
    
    	
    
        mavros_msgs::SetMode mode_cmd;
        mode_cmd.request.base_mode = 0;
        mode_cmd.request.custom_mode = target_mode;

        //成功退出
        if (set_mode_client.call(mode_cmd) && mode_cmd.response.mode_sent) {
            return true;
        }

        // 检查是否超时
        if (ros::Time::now() - start > ros::Duration(modeswitch_timeout_)) {
            ROS_WARN_STREAM("Timeout setting " << target_mode << " mode");
            return false;
        }

        ros::spinOnce();
        rate.sleep();
    }

    // 检查ROS节点是否正常运行
    if (!ros::ok()) {
        ROS_ERROR("ROS node is not running properly during setmode");
        return false;
    }

    ROS_ERROR("Modeset failed unkunown reason unexpectedly!");
    return false;
}
