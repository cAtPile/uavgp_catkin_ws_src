/**
 * @file avoid_planner.cpp
 * @brief 
 * @details 
 * @author
 * @date
 */
#include"avoid_planner_pkg/avoid_planner.h"

AvoidPlanner::AvoidPlanner(){
    
    //导入参数
    loadParams();

    //============ros============
    // 订阅点云话题
    pointcloud_sub_ = nh_.subscribe(lidar_topic_, 10,
                                   &AvoidPlanner::pointcloudCB, this);

}