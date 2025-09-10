/*
* file:     land.cpp
* name：    landSwitch
* describe：降落
* input：   NONE
* output:   true    ->  降落成功
*           false   ->  降落失败
* param:    LANDING_TIMEOUT,LANDING_TOLERANCE
* depend:   ros-noetic,cpp,mavros,px4,ununtu20.04,apoc.h
* function: armSwitch
* vision:   1.0
* method：  记录当前位置，使用flytoAbsolute/flytoPIDcorrect到当前x/y/LANDING_TOLERANCE/yaw,然后disarm
* info:     
*/

#include "apoc_pkg/apoc.h"

bool apoc::landSwitch(){
    
    float land_x = current_pose.pose.position.x;
    float land_y = current_pose.pose.position.x;
    float land_z = LANDING_TOLERANCE;
        
    tf2::Quaternion quat(
        current_position.pose.orientation.x,
        current_position.pose.orientation.y,
        current_position.pose.orientation.z,
        current_position.pose.orientation.w
    );
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, land_yaw;
    mat.getRPY(roll, pitch, land_yaw);

    ros::Time start = ros::Time::now();

    while (ros::ok() && (ros::Time::now() - start).toSec() < LANDING_TIMEOUT) {
        flytoAbsolute(land_x,land_y,land_z,land_yaw);
        ros::spinOnce();
        rate.sleep();

        if( current_pose.pose.position.z <= home_pose.pose.position.z + LANDING_TOLERANCEE){
                armSwitch(0);
            }

    }

    if ((ros::Time::now() - start).toSec() > LANDING_TIMEOUT) {
        ROS_INFO("Landing TIMEOUT,FORCE DISARM");
        armSwitch(0);
    }

    // 检查ROS节点是否正常运行
    if (!ros::ok()) {
        ROS_ERROR("ROS node is not running properly during land");
        return false;
    }

    // ROS节点异常退出
    ROS_ERROR("ROS node shutdown during land");
    return false;

}