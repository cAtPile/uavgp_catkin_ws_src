/**
 * @file avoid_planner.h
 * @brief 
 * @details 
 * @par apoc_pkg
 * @author apoc
 * @date 2025/10/7
 */

#ifndef AVOID_PLANNER_H
#define AVOID_PLANNER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <avoid_planner_pkg/AvoidPlannerAction.h>  // 自定义Action消息
#include <Eigen/Dense>
#include <memory>
#include <mutex>

#include "avoid_planner_pkg/pointcloud_processor.h"
#include "avoid_planner_pkg/potential_field.h"
#include "avoid_planner_pkg/utils.h"

#include <geometry_msgs/Vector3Stamped.h>
#include <thread>


class PointcloudProcessor;
class PotentialFieldCalculator;


/**
 * @class PlannerBridge
 * @brief apoc和avoid_planner桥
 * @details 创建一个ROS Action: 接收apoc的cmd控制avoid_planner启动停止
 *                         接收apoc的goal目标位置
 *          使用pointcloud_processor处理点云数据
 *          使用potential_field计算势场和力
 *          发布运动方向direction到apoc
 * @see class PointcloudProcessor 处理点云
 * @see class PotentialFieldCalculator 计算势场
 */
class PlannerBridge{
private:

    //===========ROS节点====================
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    //===========Action服务器================
    actionlib::SimpleActionServer<avoid_planner_pkg::AvoidPlannerAction> as_;
    std::string action_name_;
    
    //============Action消息===================
    avoid_planner_pkg::AvoidPlannerGoal current_goal_;
    avoid_planner_pkg::AvoidPlannerFeedback feedback_;
    avoid_planner_pkg::AvoidPlannerResult result_;

    //==============子模块指针================
    std::unique_ptr<PointcloudProcessor> pointcloud_processor_;
    std::unique_ptr<PotentialFieldCalculator> potential_field_calculator_;

    // 数据缓存
    PolarField current_polar_field_;
    Eigen::Vector3d current_direction_;
    std::mutex data_mutex_;  // 数据保护锁

    // 状态变量
    bool is_running_;
    ros::Rate update_rate_;

    // 回调函数
    void goalCallback();
    void pointcloudCB();
    void preemptCallback();

    
    // 主处理循环
    void processLoop();
    
    // 数据处理函数
    bool updatePointcloud();
    bool calculatePotentialField();
    void publishDirection();

public:

    PlannerBridge(const std::string& name);//构造函数
    ~PlannerBridge();//析构函数
    void start();//启动器
    Eigen::Vector3d getCurrentDir();//获取当前方向
    PolarField getCurrentPolarField();//获取当前极坐标地图
};

#endif  // AVOID_PLANNER_H