#include "mid360_avoidance/local_planner.h"
#include <ros/ros.h>

int main(int argc, char**argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "mid360_avoidance_node");
    ros::NodeHandle nh("~");  // 使用私有命名空间

    try {
        // 创建局部规划器实例
        mid360_avoidance::LocalPlanner local_planner(nh);

        // 启动局部规划器
        local_planner.start();
        ROS_INFO("Local avoidance system started");

        // 保持节点运行
        ros::spin();

        // 停止规划器
        local_planner.stop();
        ROS_INFO("Local avoidance system stopped");
    } catch (const std::exception& e) {
        ROS_FATAL("Failed to initialize local planner: %s", e.what());
        return 1;
    }

    return 0;
}