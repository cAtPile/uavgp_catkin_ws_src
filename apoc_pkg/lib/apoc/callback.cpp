
/*
*   name:       callback.cpp
*   discribe:   回调函数
*   vision:     1.0
*/

#include "apoc_pkg/apoc.h"

void apoc::state_cb(const mavros_msgs::State::ConstPtr& msg){current_state = *msg;}
void apoc::local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){current_pose = *msg;}
void apoc::detection_data_cb(const apoc_pkg::detection_data::ConstPtr& msg){current_detection = *msg;}
void apoc::lp_setvel_cb(const geometry_msgs::Twist::ConstPtr& msg){ current_lp_sv = *msg;}
void apoc::lp_setpose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){current_lp_sp = *msg;}
void apoc::lp_trajectory_cb(const mavros_msgs::Trajectory::ConstPtr& msg){current_lp_trajector y= *msg;}
void apoc::lp_obstacle_cb(const sensor_msgs::LaserScan::ConstPtr& msg){current_lp_obstacle = *msg;}
