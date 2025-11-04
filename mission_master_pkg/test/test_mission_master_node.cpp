/**
 * @file test_mission_master_node.cpp
 * @brief 任务管理主节点入口
 * @date 2025/11/2
 */
#include "mission_master_pkg/mission_master.h"
#include <ros/ros.h>

int main(int argc, char**argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "mission_master_node");
    ROS_INFO("Mission master node initialized");

    // 创建任务管理实例
    MissionMaster mission_master;

    // 主循环
    ros::Rate loop_rate(20); // 20Hz循环频率
    while (ros::ok())
    {
        // 输出当前状态信息
        mission_master.getState();

        // 处理回调函数
        ros::spinOnce();

        // 按照循环频率休眠
        loop_rate.sleep();
    }

    ROS_INFO("Mission master node exited");
    return 0;
}