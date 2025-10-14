/**
 * @file potential_field.h
 * @brief 势场构建
 * @details 构建势场和合力方向
 */
#ifndef AVOID_PLANNER_POTENTIAL_FIELD_H
#define AVOID_PLANNER_POTENTIAL_FIELD_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <cmath>

#include "avoid_planner_pkg/pointcloud_processor.h"

namespace avoid_planner {

/**
 * @class PotentialFieldCalculator
 * @brief 人工势场法计算类，用于三维空间中的避障规划
 */
class PotentialFieldCalculator {
private:

    //===========ros成员================
    ros::NodeHandle nh_;

    //===========数据成员================
    double att_gain_;           // 引力增益系数
    double rep_gain_;           // 斥力增益系数
    double rep_radius_;         // 斥力影响半径
    double max_force_;          // 最大合力限制
    double field_resolution_;   // 势场分辨率(m)
    double field_range_;        // 势场作用范围(m)
    double attract_range_;      // 引力影响范围
    Eigen::Vector3d current_pose_;  // 当前位置(x, y, z)
    Eigen::Vector3d goal_pose_;     // 目标位置(x, y, z)
    Eigen::Vector3d force_direction_;  // 合力方向
    Eigen::Vector3d current_polar_goal_;    // 目标的极坐标位置（az，el，dis）
    PotentialGrid current_field_;   // 当前势场
    PolarHistogram current_histogram_; // 当前极坐标直方图
    bool is_updated_;           // 势场是否已更新的标志

    //===========成员函数=====================
    void loadParams(); //加载参数
    PotentialGrid generatePotentialField(); //生成势场图
    PolarHistogram getPolarHistogram(); //获取直方图
    double calculateTotalForce(double az, double el,double att_distance);//计算势力,有引力情况
    double calculateRepulsiveForce(double az, double el);//仅斥力
    void updatePolarGoal();//获取目标极坐标
    Eigen::Vector3d generateTotalForce();//生成合力方向

public:

    PotentialFieldCalculator(ros::NodeHandle& nh);//构造函数
    ~PotentialFieldCalculator()= default;//析构
    PotentialGrid getPotentialField(){return current_field_;}//获取当前势场
    Eigen::Vector3d getForceDirection(){return current_field_.force_vector;}//获取当前方向
    void setGoal(double gaol_x,double goal_y,double goal_z);
    void setCurrentPose(double current_x,double current_y,double current_z);

};

}  // namespace avoid_planner

#endif  // AVOID_PLANNER_POTENTIAL_FIELD_H