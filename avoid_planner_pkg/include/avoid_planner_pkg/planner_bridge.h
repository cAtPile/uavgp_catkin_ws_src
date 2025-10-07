/**
 * @file planner_brideg.h
 * @brief 规划桥
 * @details 构建一个服务
 *          根据action,启动pointcloud_processor和potential_filed
 * @par apoc_pkg
 * @author apoc
 * @date 2025/10/7
 */

#ifndef AVOID_PLANNER_PLANNER_BRIDGE_H
#define AVOID_PLANNER_PLANNER_BRIDGE_H

# include<ros/ros.h>

namespace avoid_planner{

/**
 * @class AvoidPlannerBridge
 * @brief apoc和avoid_planner桥
 * @details 创建一个action 
 *          收apoc的cmd
 *          发运动方向
 */
class planner_bridge
{
private:
    /* data */
public:
    planner_bridge(/* args */);
    ~planner_bridge();
};

}


#endif