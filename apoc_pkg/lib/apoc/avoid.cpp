/*


*/

#include "apoc_pkg/apoc.h"

//声明
ros::Publisher goal_pub

ros::ros::Subscriber lp_setpoint_sub;
ros::ros::Subscriber lp_trajectory_sub;

lp_setpoint_sub = nh.subscribe</*msg_type*/>("/*topic_name*/", 10, /*subscribe_callback_name*/);
lp_trajectory_sub = nh.subscribe</*msg_type*/>("/*topic_name*/", 10, /*subscribe_callback_name*/);


// 发布目标点给Local Planner
ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);

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
/*
执行ego_cmd 的命令
到达检测

*/

}