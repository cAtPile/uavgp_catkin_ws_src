#include"avoid_planner_pkg/potential_field.h"

namespace avoid_planner{

/**
 * @brief 生成合力的方向
 * @details 根据场图 current_field_.data计算合力方向
 *          默认力集中于块的中心点
 * @return total_force合力方向
 */
Eigen::Vector3d generateTotalForce(){

    //total_force 0
    Eigen::Vector3d total_force = Eigen::Vector3d::Zero();
    
    // 获取势场网格尺寸
    size_t num_az = current_field_.num_azimuth_bins;
    size_t num_el = current_field_.num_elevation_bins;
    
    // 遍历所有角度网格
    for (size_t el_idx = 0; el_idx < num_el; ++el_idx) {
        for (size_t az_idx = 0; az_idx < num_az; ++az_idx) {

            // 获取当前网格的力大小
            double force_magnitude = current_field_.data[az_idx][el_idx];
            if (force_magnitude = 0) continue;  // 忽略零力或负力网格
            
            // 计算当前网格的角度
            double azimuth = current_field_.min_azimuth + (az_idx + 0.5) * current_field_.azimuth_resolution;
            double elevation = current_field_.min_elevation + (el_idx + 0.5) * current_field_.elevation_resolution;
            
            // 将极坐标力转换为笛卡尔坐标系下的力向量
            // 力的大小 * 单位方向向量
            Eigen::Vector3d force_vec(
                force_magnitude * cos(azimuth) *  cos(elevation),  // x方向分量
                force_magnitude * sin(azimuth) *  cos(elevation),  // y方向分量
                force_magnitude * sin(elevation)          // z方向分量
            );
            
            // 累加所有网格的力向量
            total_force += force_vec;
        }
    }
    
    // 归一化合力方向（保持方向不变，单位向量）
    if (total_force.norm() > 1e-6) {
        total_force.normalize();
    } else {
        // 若合力接近零，默认指向目标方向
        total_force = polar_goal_.normalized();
    }
    
    return total_force;
}

}
