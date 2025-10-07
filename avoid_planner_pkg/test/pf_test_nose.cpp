#include "avoid_planner_pkg/potential_field.h"
#include <ros/ros.h>
#include <iostream>

int main(int argc, char**argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "potential_field_test");
    ros::NodeHandle nh;

    // 创建势场计算器实例
    avoid_planner::PotentialFieldCalculator planner(nh);
    ROS_INFO("测试程序启动，初始化势场计算器");

    // 设置当前位置和目标位置（示例坐标）
    planner.setCurrentPose(0.0, 0.0, 0.0);       // 起点：原点
    planner.setGoal(5.0, 3.0, 1.5);              // 目标点：(5,3,1.5)
    ROS_INFO("设置初始位置: (0,0,0)，目标位置: (5,3,1.5)");

    // 生成势场
    avoid_planner::PotentialGrid field = planner.generatePotentialField();
    ROS_INFO("势场生成完成，网格尺寸: 方位角%d个, 俯仰角%d个",
             (int)field.num_azimuth_bins,
             (int)field.num_elevation_bins);

    // 获取合力方向并打印
    Eigen::Vector3d force_dir = planner.getForceDirection();
    ROS_INFO_STREAM("计算得到的合力方向: (" 
                   << force_dir.x() << ", " 
                   << force_dir.y() << ", " 
                   << force_dir.z() << ")");

    // 简单验证目标极坐标转换结果
    // （实际使用中需根据updatePolarGoal的逻辑验证）
    double target_dist = sqrt(pow(5.0,2) + pow(3.0,2) + pow(1.5,2));
    ROS_INFO_STREAM("目标距离理论值: " << target_dist 
                   << ", 势场本地位置: ("
                   << field.local_position.x() << ", "
                   << field.local_position.y() << ", "
                   << field.local_position.z() << ")");

    // 循环等待（模拟实时运行）
    ros::Rate rate(10);
    int count = 0;
    while (ros::ok() && count < 5) {
        // 模拟位置更新（缓慢向目标移动）
        planner.setCurrentPose(0.5*count, 0.3*count, 0.15*count);
        field = planner.generatePotentialField();
        force_dir = planner.getForceDirection();
        
        ROS_INFO_STREAM("第" << count+1 << "次更新: 合力方向 ("
                       << force_dir.x() << ", "
                       << force_dir.y() << ", "
                       << force_dir.z() << ")");
        
        count++;
        rate.sleep();
    }

    ROS_INFO("测试程序完成");
    return 0;
}