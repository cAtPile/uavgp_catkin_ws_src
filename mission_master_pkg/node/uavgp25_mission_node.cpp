#include"mission_master_pkg/mission_master.h"

int main(int argc, char**argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "mission_master_node");
    ROS_INFO("Mission master node initialized");

    // 创建任务管理实例
    MissionMaster mission_master;
    // 任务循环
    mission_master.missionExecute();

    ROS_INFO("Mission master node exited");
    return 0;
}