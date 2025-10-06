#include "avoid_planner_pkg/potential_field.h"

namespace avoid_planner {

/**
 * @brief 生成势场
 * @details 获取直方图->处理目标值->计算合力
 * @see getPolarHistogram() 获取直方图
 * @see calculateTotalForce() 计算含引力的势力
 * @see calculateRepulsiveForce() 计算仅有斥力的势力
 * @see generateTotalForce() 计算合力方向
 * @return current_field_ 当前势场
 */
PotentialGrid PotentialFieldCalculator::generatePotentialField() {

    // 获取最新的极坐标直方图
    current_histogram_ = getPolarHistogram();

    // 获取网格尺寸
    size_t el_num = current_histogram_.num_elevation_bins;
    size_t az_num = current_histogram_.num_azimuth_bins;
    
    // 初始化势场网格
    current_field_.data.resize(az_num, std::vector<double>(el_num, 0.0));
    current_field_.num_azimuth_bins = az_num;
    current_field_.num_elevation_bins = el_num;
    
    // 获取目标点
    // 将目标点转换为极坐标
    // 确定极坐标的扇区
    int goal_az;
    int goal_el;
    double att_dis;

    // 遍历所有角度网格计算势场
    for (size_t el_i = 0; el_i < el_num; ++el_i) {
        for (size_t az_i = 0; az_i < az_num; ++az_i) {

            // 如果处于有引力的扇区
            if(az_i==goal_az&&el_i==goal_el){
                double total_force = calculateTotalForce(az_i, el_i,att_dis);
            }else{
                // 计算该角度下的合力
                double total_force = calculateRepulsiveForce(az_i, el_i);
            }

            // 限制最大力，防止数值溢出
            if (total_force > max_force_) {
                total_force = max_force_;
            } else if (total_force < -max_force_) {
                total_force = -max_force_;
            }
            
            // 存储计算结果
            current_field_.data[az_i][el_i] = total_force;
        }
    }
    
    // 计算总体合力方向
    current_field_.force_vector = generateTotalForce();
    
    // 更新本地位置和时间戳
    current_field_.local_position = current_pose_;
    current_field_.timestamp = ros::Time::now();
    
    // 标记势场已更新
    is_updated_ = true;
    
    return current_field_;
}

}  // namespace avoid_planner
    