
/*
* file：    fly_re.cpp
* name：    flytoRelative
* path:     /lib
* describe：飞行到相对home位姿的坐标fly_ab_x/y/z/yaw
* input：   fly_re_x/y/z/yaw
* output:   true    ->  到达
*           false   ->  未到达
* param:    NONE
* depend:   ros-noetic,cpp,mavros,px4,ununtu20.04,apoc.h
* function: reachCheck，flytoAbsolute/flytoPIDcorrect
* vision:   1.0
* method：  fly_ab_x/y/z/yaw+home的位置姿态，使用flytoAbsolute/flytoPIDcorrect到目的地
* info:     "ROS node shutdown during flight relative"
*           相对飞行退出
*/

#include "apoc_pkg/apoc.h"

bool apoc::flytoRelative(float fly_re_x , float fly_re_y , float fly_re_z , float fly_re_yaw ){

    float ab_x = fly_re_x + home_pose.pose.position.x;
    float ab_y = fly_re_y + home_pose.pose.position.y;
    float ab_z = fly_re_z + home_pose.pose.position.z;

    // 从home位置的四元数计算偏航角(弧度)
    tf2::Quaternion q(
        home_pose.pose.orientation.x,
        home_pose.pose.orientation.y,
        home_pose.pose.orientation.z,
        home_pose.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, home_yaw;
    m.getRPY(roll, pitch, home_yaw);

    // 计算相对home偏航角的绝对偏航角
    float ab_yaw = fly_re_yaw + home_yaw;
   
    if (flytoAbsolute(ab_x,ab_y,ab_z,ab_yaw)){
        return true;
    }

    // ROS节点异常退出
    ROS_ERROR("ROS node shutdown during flight relative");
    return false;
}
