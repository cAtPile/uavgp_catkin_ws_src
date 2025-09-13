/*
* file:     hover.cpp
* name：    hoverSwitch
* path:     /lib
* describe：悬停
* input：   hover_time
* output:   true    ->  悬停成功
*           false   ->  悬停失败
* param:    NONE
* depend:   ros-noetic,cpp,mavros,px4,ununtu20.04,apoc.h
* function: NONE
* vision:   1.0
* method：  维持发布当前位置
* info:     "ROS node shutdown during hover"
*           盘旋时ros节点异常
*/

#include "apoc_pkg/apoc.h"
/*

bool apoc::hoverSwitch(float hover_time){

    geometry_msgs::PoseStamped hover_pose;

    hover_pose.header.frame_id = "map";
    hover_pose.pose.position.x = current_pose.pose.position.x;
    hover_pose.pose.position.y = current_pose.pose.position.y;
    hover_pose.pose.position.z = current_pose.pose.position.z;
    hover_pose.pose.orientation.x = current_pose.pose.orientation.x;
    hover_pose.pose.orientation.y = current_pose.pose.orientation.y;
    hover_pose.pose.orientation.z = current_pose.pose.orientation.z;
    hover_pose.pose.orientation.w = current_pose.pose.orientation.w;

    ros::Time start = ros::Time::now();

    while (ros::ok() && (ros::Time::now() - start).toSec() < hover_time) {
        local_pos_pub.publish(hover_pose);
        ros::spinOnce();
        rate.sleep();
    }

    if ((ros::Time::now() - start).toSec() > hover_time) {
        return true;
    }
        
    // 检查ROS节点是否正常运行
    if (!ros::ok()) {
        ROS_ERROR("ROS node is not running properly during hover");
        return false;
    }

    // ROS节点异常退出
    ROS_ERROR("ROS node shutdown during hover");
    return false;

}*/
bool apoc::hoverSwitch(float hover_time){

    geometry_msgs::PoseStamped hover_pose;
    // 设置正确的坐标帧
    hover_pose.header.frame_id = "map";  // 或根据你的系统使用"local_origin"
    
    // 获取初始悬停位置
    hover_pose.pose.position.x = current_pose.pose.position.x;
    hover_pose.pose.position.y = current_pose.pose.position.y;
    hover_pose.pose.position.z = current_pose.pose.position.z;
    hover_pose.pose.orientation = current_pose.pose.orientation;

    ros::Time start = ros::Time::now();
    // 使用较高的控制频率，建议至少30Hz
    ros::Rate hover_rate(50);  // 50Hz的发布频率

    while (ros::ok()) {
        // 检查悬停时间是否已到
        if ((ros::Time::now() - start).toSec() >= hover_time) {
            return true;
        }
        
        // 更新时间戳（非常重要，否则MAVROS会忽略过期指令）
        hover_pose.header.stamp = ros::Time::now();
        
        // 发布悬停位置指令
        local_pos_pub.publish(hover_pose);
        
        ros::spinOnce();
        hover_rate.sleep();
    }

    // ROS节点异常退出
    ROS_ERROR("ROS node shutdown during hover");
    return false;
}
