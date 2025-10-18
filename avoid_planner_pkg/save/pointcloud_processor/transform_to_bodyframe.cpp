#include "avoid_planner_pkg/pointcloud_processor.h"
#include "avoid_planner_pkg/utils.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <limits>

namespace avoid_planner {
/**
 * @brief 将点云从激光雷达坐标系转换到机体坐标系
 * @details 通过TF变换获取激光雷达与机体坐标系之间的位姿关系，
 *          并将输入点云中的所有点应用该变换，得到机体坐标系下的点云
 * 
 * @param[in] input 激光雷达坐标系下的点云数据（pcl::PointXYZ类型）
 * @param[out] output 转换后机体坐标系下的点云数据（输出参数）
 * 
 * @return 转换成功返回true，失败返回false
 * 
 * @note 函数使用tf2库进行坐标变换，支持平移和旋转的复合变换
 * @note 变换超时时间设置为0.1秒，避免长时间阻塞
 * @note 若变换获取失败，会通过ROS_WARN_THROTTLE输出警告信息（1秒内最多一次）
 * 
 * @see tf_buffer_ TF变换缓冲区，用于查询坐标系间的变换关系
 * @see body_frame_id_ 机体坐标系ID（目标坐标系）
 * @see lidar_frame_id_ 激光雷达坐标系ID（源坐标系）
 * @see pcl::transformPointCloud() PCL库中点云变换函数，批量处理点云
 * @see tf2::transformToEigen() 将TF变换转换为Eigen矩阵的工具函数
 * 
 * @warning 若TF变换查找失败（如坐标系未发布或名称错误），会返回false且output为空
 * @warning 输入点云为空时仍会尝试执行变换，但输出也将为空
 */
bool PointcloudProcessor::transformToBodyFrame(const pcl::PointCloud<pcl::PointXYZ>& input, 
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

}