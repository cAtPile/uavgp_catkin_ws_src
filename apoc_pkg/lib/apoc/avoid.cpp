/*


*/

#include "apoc_pkg/apoc.h"

/*apoc.h*/
//声明pub
ros::Publisher goal_pub;
ros::Publisher local_pos_pub;
ros::Publisher local_vel_pub;
ros::Publisher mav_trajectory_sub;
ros::Publisher mav_obstacle_sub;

//声明sub
ros::Subscriber lp_setpose_sub;
ros::Subscriber lp_setvel_sub;
ros::Subscriber lp_trajectory_sub;
ros::Subscriber lp_obstacle_sub;

//缓存区
geometry_msgs::Twist current_lp_sv;
geometry_msgs::PoseStamped current_lp_sp;
mavros_msgs::Trajectory current_lp_trajectory;
sensor_msgs::LaserScan current_lp_obstacle;

/*apoc.cpp*/
//订阅器
lp_setvel_sub = nh_.advertise<geometry_msgs::Twist>("apoc/setpoint_velocity/cmd_vel_unstamped", 10, lp_setvel_cb,this);
lp_setpose_sub = nh_.advertise<geometry_msgs::PoseStamped>("apoc/setpoint_position/local", 10, lp_setpose_cb,this);
lp_trajectory_sub = nh_.advertise<mavros_msgs::Trajectory>("apoc/trajectory/generated", 10,lp_trajectory_cb,this);
lp_obstacle_sub = nh_.advertise<sensor_msgs::LaserScan>("apoc/obstacle/send", 10,lp_obstacle_cb,this);

//发布器
goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);//已存在
local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);//已存在
local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10); //已存在
mav_trajectory_pub = nh_.advertise<mavros_msgs::Trajectory>("mavros/trajectory/generated", 10);
mav_obstacle_pub = nh_.advertise<sensor_msgs::LaserScan>("mavros/obstacle/send", 10);

//回调函数
void lp_setvel_cb(const geometry_msgs::Twist::ConstPtr& msg){ current_lp_sv = *msg;}
void lp_setpose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){current_lp_sp = *msg;}
void lp_trajectory_cb(const mavros_msgs::Trajectory::ConstPtr& msg){current_lp_trajector y= *msg;}
void lp_obstacle_cb(const sensor_msgs::LaserScan::ConstPtr& msg){current_lp_obstacle = *msg;}

bool apoc::avoidGoal(float goal_x,float goal_y,float goal_z){
    
    //临时参数
    float GOAL_TIMEOUT = 30 ;//导航超时

    //设置目标点
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "local_origin";  // 与Local Planner坐标系一致
    goal_pose.pose.position.x = goal_x;
    goal_pose.pose.position.y = goal_y;
    goal_pose.pose.position.z = goal_z;
    goal_pose.pose.orientation.x = 0;
    goal_pose.pose.orientation.y = 0;
    goal_pose.pose.orientation.z = 0;
    goal_pose.pose.orientation.w = 1;

    //发布目标点
    goal_pub.publish(goal_pose);
    
    //建立局部消息
    geometry_msgs::Twist avoid_sv;
    geometry_msgs::PoseStamped avoid_sp;
    mavros_msgs::Trajectory avoid_trajectory;
    sensor_msgs::LaserScan avoid_obstacle;

    //超时处理
    ros::Time start_time = ros::Time::now();
    const ros::Duration timeout(GOAL_TIMEOUT);
    //
    while (ros::ok()) {

        //消息传递
        avoid_sv = current_lp_sv;
        avoid_sp = current_lp_sp;
        avoid_trajectory = current_lp_trajectory;
        avoid_obstacle = current_lp_obstacle;

        //时间戳更新
        avoid_sp.header.stamp = ros::Time::now();
        avoid_sv.header.stamp = ros::Time::now();
        avoid_trajectory.header.stamp = ros::Time::now();
        avoid_obstacle.header.stamp = ros::Time::now();

        //
        avoid_sv.twist = current_lp_sv;

        //发布
        local_pos_pub.publish(avoid_sv);
        local_vel_pub.publish(avoid_sp);
        mav_trajectory_pub.publish(avoid_trajectory);
        mav_obstacle_pub.publish(avoid_obstacle);

        // 检查是否到达目标位置
        if (reachCheck(fly_ab_x, fly_ab_y, fly_ab_z, fly_ab_yaw)) {
            return true;
        }

        // 检查是否超时
        if ((ros::Time::now() - start_time) > timeout) {
            ROS_WARN_STREAM("Timeout while flying to goal position. Time elapsed: " 
                          << (ros::Time::now() - start_time).toSec() << "s");
            return false;
        }

        ros::spinOnce();
        rate.sleep();
    }

}