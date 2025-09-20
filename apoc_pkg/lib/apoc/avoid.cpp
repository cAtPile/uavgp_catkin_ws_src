/*


*/

#include "apoc_pkg/apoc.h"

//声明
ros::Publisher goal_pub
ros::ros::Subscriber ego_cmd;
ego_cmd = nh.subscribe</*msg_type*/>("/*topic_name*/", 10, /*subscribe_callback_name*/);


// 发布目标点给Local Planner
ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

void apoc::egocmd_cb(){

    local_pos_pub.publish(
}

bool apoc::avoidGoal(float goal_x,float goal_y,float goal_z){
    
    //临时参数
    float GOAL_TIMEOUT = 30 ;

    //
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "local_origin";  // 与Local Planner坐标系一致
    goal_pose.pose.position.x = goal_x;
    goal_pose.pose.position.y = goal_y;
    goal_pose.pose.position.z = goal_z;
    goal_pose.pose.orientation.x = 0;
    goal_pose.pose.orientation.y = 0;
    goal_pose.pose.orientation.z = 0;
    goal_pose.pose.orientation.w = 1;

    //
    ros::Time start_time = ros::Time::now();
    const ros::Duration timeout(GOAL_TIMEOUT);  // 30秒超时时间

    while (ros::ok()) {

        // 更新消息时间戳
        goal_pose.header.stamp = ros::Time::now();

        // 发布目标位置
        goal_pub.publish(goal_pose);

        // 检查是否到达目标位置
        if ((reachCheck(goal_x, goal_y, goal_z, 0)) {
            return true;
        }

        // 检查是否超时
        if ((ros::Time::now() - start_time) > timeout) {
            ROS_WARN_STREAM("Timeout while flying to avoid goal position. Time elapsed: " 
                          << (ros::Time::now() - start_time).toSec() << "s");
            return false;
        }

        ros::spinOnce();
        rate.sleep();
    }

}