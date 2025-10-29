#include "avoid_planner_pkg/avoid_planner.h"
#include "avoid_planner_pkg/PolarFieldMsg.h"  
#include <ros/ros.h>

int main(int argc, char**argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "polar_field_publisher_test");
    ros::NodeHandle nh("~");

    try {
        // 创建规划器实例
        AvoidPlanner planner;
        ROS_INFO("AvoidPlanner Buided!");

        // 创建消息发布者，发布极坐标场数据
        ros::Publisher field_pub = nh.advertise<avoid_planner_pkg::PolarFieldMsg>(
            "/polar_field_test", 10);  // 话题名：/polar_field_test

        // 循环获取并发布数据
        ros::Rate rate(10);  // 10Hz发布频率
        int count = 0;
        while (ros::ok()) {
            // 调用公有接口获取极坐标场数据
            PolarField field = planner.getCurrentPolarField();

            // 构造消息
            avoid_planner_pkg::PolarFieldMsg msg;

            // 填充消息头部
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "base_link";  // 假设使用机体坐标系

            // 填充极坐标场核心参数
            msg.azimuth_resolution = field.azimuth_resolution;
            msg.elevation_resolution = field.elevation_resolution;
            msg.min_azimuth = field.min_azimuth;
            msg.max_azimuth = field.max_azimuth;
            msg.min_elevation = field.min_elevation;
            msg.max_elevation = field.max_elevation;

            // 填充传感器范围参数
            msg.min_range = field.min_range;
            msg.max_range = field.max_range;

            // 填充网格数量
            msg.num_azimuth_bins = field.num_azimuth_bins;
            msg.num_elevation_bins = field.num_elevation_bins;

            //
            // 填充障碍物距离数组 
            msg.obstacle_distances.clear();  // 清空原有数据
            for (const auto& az_bin : field.dis_map) {  // 遍历方位角维度
                for (double distance : az_bin) {         // 遍历仰角维度
                    // 将无穷大值替换为最大范围
                    if (std::isinf(distance)) {
                        msg.obstacle_distances.push_back(field.max_range);
                    } else {
                        msg.obstacle_distances.push_back(distance);
                    }
                }
            }

            // 填充势场数组 (pot_map)
            msg.pot_map.clear();  // 清空原有数据
            for (const auto& az_bin : field.pot_map) {  // 遍历方位角维度
                for (double pot_value : az_bin) {       // 遍历仰角维度
                    msg.pot_map.push_back(pot_value);
                }
            }

            // 发布消息
            field_pub.publish(msg);
            ros::spinOnce();
            rate.sleep();
            count++;
        }


        ROS_INFO("Successfully");
    } catch (const std::exception& e) {
        ROS_ERROR("field: %s", e.what());
        return 1;
    }

    return 0;
}