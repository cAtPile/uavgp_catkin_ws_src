#include "avoid_planner_pkg/avoid_planner.h"
#include <ros/ros.h>

int main(int argc, char**argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "avoid_planner_test");
    ros::NodeHandle nh;

    try {
        // 尝试创建规划器实例（测试构造函数）
        AvoidPlanner planner;
        ROS_INFO("AvoidPlanner 实例创建成功");

        // 循环获取极坐标场数据（测试公有接口）
        ros::Rate rate(10); // 10Hz
        int count = 0;
        while (ros::ok() && count < 10) { // 只运行10次
            // 调用公有接口获取数据
            PolarField field = planner.getCurrentPolarField();
            
            // 打印基本信息（验证数据获取正常）
            ROS_INFO("第%d次获取数据: 方位角网格数=%zu, 仰角网格数=%zu",
                     count+1,
                     field.num_azimuth_bins,
                     field.num_elevation_bins);

            ros::spinOnce();
            rate.sleep();
            count++;
        }

        ROS_INFO("测试完成");
    } catch (const std::exception& e) {
        ROS_ERROR("测试失败: %s", e.what());
        return 1;
    }

    return 0;
}