#include"avoid_planner_pkg/potential_field.h"

namespace avoid_planner{

/**
 * @brief 更新极坐标目标位置
 * @details 将目标位置goal_pose_(xyz)
 *          和当前的位置current_pose_(xyz)
 *          转换为相对的极坐标位置，
 *          并更新到current_polar_goal_(az,el,dis)
 */
void PotentialFieldCalculator::updatePolarGoal(){
    
    // 相对位置 NEU
    Eigen::Vector3d rel_pos = goal_pose_ - current_pose_;
    
    // 计算距离 dis
    double distance = rel_pos.norm();
    if (distance < 1e-6) {  // 避免距离为0导致的计算异常
        polar_goal_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        return;
    }
    
    // 计算方位角az
    double azimuth = atan2(rel_pos.y(), rel_pos.x());
    
    // 计算俯仰角el
    double elevation = asin(rel_pos.z() / distance);
    
    // 存储极坐标结果（az方位角，el俯仰角，dis距离）
    polar_goal_ = Eigen::Vector3d(azimuth, elevation, distance);

}

}