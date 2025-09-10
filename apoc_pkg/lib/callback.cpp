
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
    current_position = *msg;
}