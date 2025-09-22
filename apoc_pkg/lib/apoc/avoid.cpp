/*


*/

#include "apoc_pkg/apoc.h"

//声明
ros::Publisher goal_pub;
ros::Publisher local_pos_pub;
ros::Publisher local_vel_pub;
ros::Publisher mav_trajectory_sub;
ros::Publisher mav_obstacle_sub;

ros::ros::Subscriber lp_setpose_sub;
ros::ros::Subscriber lp_setvel_sub;
ros::ros::Subscriber lp_trajectory_sub;
ros::ros::Subscriber lp_obstacle_sub;

lp_setvel_sub = nh_.advertise<geometry_msgs::Twist>("apoc/setpoint_velocity/cmd_vel_unstamped", 10, lp_setvel_cb);
lp_setpose_sub = nh_.advertise<geometry_msgs::PoseStamped>("apoc/setpoint_position/local", 10, lp_setpose_cb);
lp_trajectory_sub = nh_.advertise<mavros_msgs::Trajectory>("apoc/trajectory/generated", 10,lp_trajectory_cb) ;
lp_obstacle_sub = nh_.advertise<sensor_msgs::LaserScan>("apoc/obstacle/send", 10,lp_obstacle_cb);

// 发布目标点给Local Planner
goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10); 
mav_trajectory_pub = ;
mav_obstacle_pub = ;

void lp_setvel_cb(){}
void lp_setpose_cb(){}
void lp_trajectory_cb(){}
void lp_obstacle_cb(){}

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