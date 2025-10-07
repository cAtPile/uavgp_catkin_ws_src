#include "avoid_planner_pkg/potential_field.h"

namespace avoid_planner {

/**
 * @brief 构造函数实现
 * @param nh ROS节点句柄，用于加载参数
 */
PotentialFieldCalculator::PotentialFieldCalculator(ros::NodeHandle& nh) 
  : att_gain_(1.0), 
    rep_gain_(10.0), 
    rep_radius_(2.0), 
    max_force_(50.0), 
    field_resolution_(0.5), 
    field_range_(10.0), 
    attract_range_(50.0),
    current_pose_(Eigen::Vector3d::Zero()),
    goal_pose_(Eigen::Vector3d::Zero()),
    force_direction_(Eigen::Vector3d::Zero()),
    polar_goal_(Eigen::Vector3d::Zero()),
    is_updated_(false) {

    //加载参数
    loadParams();

    //参数展示
    ROS_INFO("att_gain_ = %1.f",att_gain_);
    ROS_INFO("rep_gain_ = %1.f",rep_gain_);
    ROS_INFO("rep_radius_ = %1.f",rep_radius_);
    ROS_INFO("max_force_ = %1.f",max_force_);
    ROS_INFO("field_resolution_ = %1.f",field_resolution_);
    ROS_INFO("field_range_ = %1.f",field_range_);
    ROS_INFO("attract_range_ = %1.f",attract_range_);

    // 初始化极坐标直方图
    current_histogram_ = PolarHistogram();
    
    // 初始化势场网格
    current_field_ = PotentialGrid();
    
    ROS_INFO("PotentialFieldCalculator initialized successfully");
}

}  // namespace avoid_planner