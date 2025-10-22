/**
 * @file tfBodyframe.cpp
 * @brief 坐标系转换
 * @details
 * @author apoc
 * @date 2025/10/17
 */
#include"avoid_planner_pkg/avoid_planner.h"

/**
 * @brief 将点云从激光雷达坐标系转换到机体坐标系
 * @details 通过TF变换获取激光雷达与机体坐标系之间的位姿关系，
 *          并将输入点云中的所有点应用该变换，得到机体坐标系下的点云
 * @param[in] input 激光雷达坐标系下的点云数据（pcl::PointXYZ类型）
 * @param[out] output 转换后机体坐标系下的点云数据（输出参数）
 * @return 转换成功返回true，失败返回false
 */
bool AvoidPlanner::tfBodyframe(const pcl::PointCloud<pcl::PointXYZ>& input, 
                                             pcl::PointCloud<pcl::PointXYZ>& output) {
    try {
        // 获取激光雷达到机体坐标系的变换
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            body_frame_id_, lidar_frame_id_, ros::Time(0), ros::Duration(0.1));
        
        // 转换为Eigen变换
        Eigen::Affine3d eigen_transform = tf2::transformToEigen(transform);
        
        // 应用变换到整个点云
        pcl::transformPointCloud(input, output, eigen_transform);
        
        return true;
    }
    catch (tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "Transform exception: %s", ex.what());
        return false;
    }
}
