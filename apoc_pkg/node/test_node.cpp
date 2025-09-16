#include "apoc_pkg/apoc.h"
#include "apoc_pkg/pidctrl.h"
#include <ros/ros.h>

// Forward declaration
bool missionPlanner(apoc& apoc_control);

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "apoc_complete_test_node", ros::init_options::AnonymousName);
    ROS_INFO("=== APOC Complete Flight Test Node Started ===");

    // 创建ROS节点句柄
    ros::NodeHandle nh("~");  // 私有命名空间
    // 创建apoc类实例
    apoc apoc_control;

    // 初始化主循环频率
    ros::Rate main_rate(20);
    ros::Time test_start_time = ros::Time::now();

    // 连接、解锁并切换模式
    if (ros::ok() && apoc_control.connectSwitch(1) && 
        apoc_control.armSwitch(1) && apoc_control.modeSwitch(1)) {
        ROS_INFO("System initialized successfully, starting mission...");
        
        // 执行任务规划
        bool mission_success = missionPlanner(apoc_control);
        if (!mission_success) {
            ROS_ERROR("Mission failed, initiating landing...");
        }
    } else {
        ROS_ERROR("Failed to initialize system properly");
    }

    // 执行降落
    bool land_success = apoc_control.landSwitch();
    if (land_success) {
        ROS_INFO("Landing completed successfully");
    } else {
        ROS_ERROR("Landing failed");
    }

    ROS_INFO("Mission end");

    return 0;
}
    
// 任务规划器
bool missionPlanner(apoc& apoc_control) {
    ROS_INFO("Starting mission sequence...");
    
    // 起飞
    if (!apoc_control.takeoffSwitch(1)) {
        ROS_ERROR("Takeoff failed");
        return false;
    }
    ROS_INFO("Takeoff successful");

    // 飞到相对位置
    if (!apoc_control.flytoRelative(1, 0, 1, 0)) {
        ROS_ERROR("Failed to reach target position");
        return false;
    }
    ROS_INFO("Reached target position");

    // 悬停
    if (!apoc_control.hoverSwitch(3)) {
        ROS_ERROR("Hover mode failed");
        return false;
    }
    ROS_INFO("Hover sequence completed");

    return true;
}