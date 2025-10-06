# include "avoid_planner_pkg/potential_field.h"

namespace{
    
/**
 * @brief 计算势力，无引力状态
 * @details 处理斥力场，计算该点斥力
 * @param az 方位角索引
 * @param el 俯仰角索引
 * @return force 该点的力
 */
double PotentialFieldCalculatorz::calculateRepulsiveForce(double az, double el, ){

    float rep_dis = current_histogram_.data[az][el];
    return ep_gain_/rep_dis;
    
}

}