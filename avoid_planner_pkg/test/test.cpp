// 点云测试程序（修正版）
#include "avoid_planner_pkg/avoid_planner.h"
#include "avoid_planner_pkg/PolarFieldMsg.h"  // 自定义的PolarField消息类型
#include <ros/ros.h>

int main(int argc, char**argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "polar_field_publisher");
    ros::NodeHandle nh("~");

    try {
        // 创建避障规划器实例
        AvoidPlanner planner;
        
        // 创建PolarField消息发布者
        ros::Publisher field_pub = nh.advertise<avoid_planner_pkg::PolarFieldMsg>(
            "/polar_field", 10);

        ROS_INFO("Polar field publisher node initialized.");

        // 主循环
        ros::Rate rate(20);  // 20Hz循环频率
        while (ros::ok()) {
            // 1. 通过公有接口获取锁（替代直接访问私有mutex）
            std::lock_guard<std::mutex> lock(planner.getUpdatedMutex());
            
            // 2. 通过公有接口检查更新标志（替代直接访问私有is_updated_）
            if (planner.checkIsUpdated()) {
                // 获取极坐标场数据
                const PolarField& field = planner.getCurrentPolarField();  // 修正命名空间
                
                // 构造ROS消息
                avoid_planner_pkg::PolarFieldMsg msg;
                
                // 填充消息头部
                msg.header.stamp = field.timestamp;
                msg.header.frame_id = "base_link";  // 机体坐标系
                
                // 填充角度参数
                msg.azimuth_resolution = field.azimuth_resolution;
                msg.elevation_resolution = field.elevation_resolution;
                msg.min_azimuth = field.min_azimuth;
                msg.max_azimuth = field.max_azimuth;
                msg.min_elevation = field.min_elevation;
                msg.max_elevation = field.max_elevation;
                
                // 填充传感器参数
                msg.max_range = field.max_range;
                msg.min_range = field.min_range;
                
                // 填充网格尺寸
                msg.num_azimuth_bins = field.num_azimuth_bins;
                msg.num_elevation_bins = field.num_elevation_bins;
                
                // 扁平化障碍物距离数据（注意：原结构体中是dis_map，不是obstacle_distances）
                msg.obstacle_distances.clear();
                msg.obstacle_distances.reserve(
                    field.num_azimuth_bins * field.num_elevation_bins);
                for (const auto& az_bin : field.dis_map) {  // 修正为dis_map（原结构体定义）
                    msg.obstacle_distances.insert(
                        msg.obstacle_distances.end(), az_bin.begin(), az_bin.end());
                }
                
                // 扁平化势场数据
                msg.pot_map.clear();
                msg.pot_map.reserve(
                    field.num_azimuth_bins * field.num_elevation_bins);
                for (const auto& az_bin : field.pot_map) {
                    msg.pot_map.insert(
                        msg.pot_map.end(), az_bin.begin(), az_bin.end());
                }
                
                // 填充合力向量和本地位置
                msg.force_vector.x = field.force_vector.x();
                msg.force_vector.y = field.force_vector.y();
                msg.force_vector.z = field.force_vector.z();
                msg.local_position.x = field.local_position.x();
                msg.local_position.y = field.local_position.y();
                msg.local_position.z = field.local_position.z();
                
                // 发布消息
                field_pub.publish(msg);
                ROS_INFO_THROTTLE(1.0, "Published polar field with %zu x %zu bins",
                                  field.num_azimuth_bins, field.num_elevation_bins);
                
                // 3. 通过公有接口重置更新标志（替代直接修改私有is_updated_）
                planner.resetIsUpdated();
            }

            // 处理回调函数
            ros::spinOnce();
            rate.sleep();
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Node failed: %s", e.what());
        return 1;
    }

    return 0;
}