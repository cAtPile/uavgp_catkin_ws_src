
/*
*   name:       callback.cpp
*   discribe:   回调函数
*   vision:     1.0
*/

#include "apoc_pkg/apoc.h"

void apoc::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void apoc::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

// 回调函数：接收检测到的目标信息
void apoc::detection_data_cb(const apoc_pkg::Detection::ConstPtr& msg){
    current_detection = *msg;
}