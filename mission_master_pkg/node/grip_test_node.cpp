#include"mission_master_pkg/mission_master.h"

int main(int argc, char**argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "grip_test_node");
    ROS_INFO("Grip test node initialized");

    // 创建任务管理实例
    MissionMaster mission_master;
    // 任务循环
    mission_master.gripPick();

    ROS_INFO("Grip test node exited");
    return 0;
}