# include "avoid_planner_pkg/potential_field.h"

namespace{
    
/**
 * @brief 计算势力，含引力状态
 * @details 处理引力场/斥力场，计算该点合力
 * @param az 方位角索引
 * @param el 俯仰角索引
 * @param att_dis 该方向上引力源的位置
 * @return force 该点的力
 */
double PotentialFieldCalculatorz::calculateTotalForce(double az, double el, double att_dis){

    float rep_dis = current_histogram_.data[az][el];

    if(rep_dis<att_gain_){
        double force = att_gain_/att_dis-rep_gain_/rep_dis //计算出势
    }else{
        double force = att_gain_/att_dis
    }

    return force;
}

}