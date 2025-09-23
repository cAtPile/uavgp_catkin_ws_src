//aviod_planner.h
//主节点

//任务1.订阅apoc/setgoal/data话题
//      apoc/setgoal/data话题类型：
//          apoc_setgoal_pose.msg
//              float16 goal_x
//              float16 goal_y
//              float16 goal_z

//任务2.订阅apoc/setgoal/action话题
//      apoc/setgoal/action话题类型：bool

//任务3.根据输入的目标点执行localPlanner(goal_x,goal_y,goal_z)
//      规划路径并输出下一步要执行的定点飞行发布到/local_planner/setpoint,

#ifndef AVOID_PLANNER_H
#define AVOID_PLANNER_H

#include <ros/ros.h>
#include <apoc_msgs/apoc_setgoal_pose.h>  // 假设消息包名为apoc_msgs
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include "local_planner/local_planner.h"  // 复用已有路径规划器
#include <geometry_msgs/PoseStamped.h>//定点

namespace avoidance {

class AvoidPlanner {
public:
    AvoidPlanner(ros::NodeHandle& nh);
    ~AvoidPlanner() = default;

private:
    // 回调函数
    void goalDataCallback(const apoc_msgs::apoc_setgoal_pose::ConstPtr& msg);
    void goalActionCallback(const std_msgs::Bool::ConstPtr& msg);

    // 路径规划执行函数
    void executePlanner();

    ros::NodeHandle nh_;

    // 订阅器
    ros::Subscriber goal_data_sub_;    // 订阅目标点数据
    ros::Subscriber goal_action_sub_;  // 订阅目标点动作指令
    ros::Subscriber local_pos_sub;     //订阅现位置

    // 发布器
    ros::Publisher setpoint_pub_;      // 发布避障定点

    // 内部状态
    LocalPlanner local_planner_;       // 复用已有本地规划器
    Eigen::Vector3f current_goal_;     // 当前目标点
    bool has_new_goal_ = false;        // 是否有新目标点
    bool start_planning_ = false;      // 是否开始规划（由action触发）
};

}  // namespace avoidance

#endif  // AVOID_PLANNER_H