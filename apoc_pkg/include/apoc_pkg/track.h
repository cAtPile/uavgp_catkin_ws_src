//
#ifndef TRACK_H
#define TRACK_H

#include <ros/ros.h>//ros
#include <geometry_msgs/PoseStamped.h>//mav定点
#include <mavros_msgs/State.h>//状态
#include <tf2/LinearMath/Quaternion.h>//四元数
#include <tf2/LinearMath/Matrix3x3.h>//rpy

#include <vector>
#include <cmath>

#include <geometry_msgs/TwistStamped.h>//mav速度

class track{
private:
    ros::Subscriber trace_sub;

public:

};

#endif
