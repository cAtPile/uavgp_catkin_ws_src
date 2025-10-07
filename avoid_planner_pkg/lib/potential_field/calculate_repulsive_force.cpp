# include "avoid_planner_pkg/potential_field.h"

namespace avoid_planner{
    
/**
 * @brief 计算势力，无引力状态
 * @details 处理斥力场，计算该点斥力
 * @param az 方位角索引
 * @param el 俯仰角索引
 * @return force 该点的力
 */
double PotentialFieldCalculatorz::calculateRepulsiveForce(double az, double el ){

    double rep_dis = current_histogram_.data[az][el];
    if (rep_dis < rep_radius_ && rep_dis != INFINITY) {
        return rep_gain_ * (1.0 / rep_dis - 1.0 / rep_radius_) / (rep_dis * rep_dis);
    }
    return 0.0;
    
}

}