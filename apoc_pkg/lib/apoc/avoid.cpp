#include "apoc_pkg/apoc.h"

/**
 * 带重构
 * @brief 
 * @details 
 */
bool apoc::avoidGoal(float goal_x,float goal_y,float goal_z){
    
    //临时参数
    float GOAL_TIMEOUT = 30 ;//导航超时

    //设置目标点
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "local_origin";  // 与Local Planner坐标系一致
    goal_pose.pose.position.x = goal_x;
    goal_pose.pose.position.y = goal_y;
    goal_pose.pose.position.z = goal_z;
    goal_pose.pose.orientation.x = flight_orien_x_;
    goal_pose.pose.orientation.y = flight_orien_y_;
    goal_pose.pose.orientation.z = flight_orien_z_;
    goal_pose.pose.orientation.w = flight_orien_w_;

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