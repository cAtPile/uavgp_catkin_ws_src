#include "apoc_pkg/apoc.h"

void apoc::landSwitch() {

    // 记录降落起始点
    float land_x = current_pose.pose.position.x;
    float land_y = current_pose.pose.position.y;
    float current_z = current_pose.pose.position.z;

    geometry_msgs::PoseStamped land_pose;

    land_pose.pose.position.x = land_x;
    land_pose.pose.position.y = land_y;
    land_pose.pose.position.z = home_pose.pose.position.z + landing_tolerance_;
    land_pose.pose.orientation.x = 0;
    land_pose.pose.orientation.y = 0;
    land_pose.pose.orientation.z = 0;
    land_pose.pose.orientation.w = 1;

    ros::Time start_time = ros::Time::now();

    while (ros::ok()){

        //
        local_pos_pub.publish(land_pose);

        if (current_pose.pose.z<=home_pose.pose.position.z + landing_tolerance_){
            armSwitch(0);
            ROS_INFO("land successful");
            break;
        }else{
            if((ros::Time::now()-start_time).toSec() <= landing_timeout_ ){
                continue;
            }else{
                armSwitch(0);
                ROS_WARN("land timeout");
                break;
            }
        }
        
        // 处理回调并控制频率
        ros::spinOnce();
        rate.sleep();
    }
 
}