#include "apoc_pkg/apoc.h"
#include "apoc_pkg/pidctrl.h"
#include <ros/ros.h>

// 任务规划器
void missionPlanner(apoc& apoc_control) {
    //已经包含连接，解锁，模式切换
    apoc_control.takeoffSwitch(1);
    apoc_control.trackSwitch();
    ROS_INFO("开始降落");
    //apoc_control.flytoRelative(1,1,1,0);
    //apoc_control.hoverSwitch(1.0);
    apoc_control.landSwitch();
}

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "apoc_complete_test_node", ros::init_options::AnonymousName);
    ROS_INFO("=== APOC Complete Flight Test Node Started ===");

    // 创建ROS节点句柄
    ros::NodeHandle nh("~");  
    // 创建apoc类实例
    apoc apoc_control;

    // 初始化主循环频率
    ros::Rate main_rate(20);
    ros::Time test_start_time = ros::Time::now();

    // 连接、解锁并切换模式
    if (ros::ok() && apoc_control.connectSwitch() && 
        apoc_control.armSwitch(1) && apoc_control.modeSwitch(1)) {
        ROS_INFO("System initialized successfully, starting mission...");
        
        // 执行任务规划
        missionPlanner(apoc_control);

    } else {
        ROS_ERROR("Failed to initialize system properly");
    }

    ROS_INFO("Mission end");

    return 0;
}