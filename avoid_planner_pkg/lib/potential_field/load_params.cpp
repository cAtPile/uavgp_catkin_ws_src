#include "avoid_planner_pkg/potential_field.h"

namespace avoid_planner {

void PotentialFieldCalculator::loadParams(){
    // 假设类内有ros::NodeHandle nh_成员
    const std::string ns = "avoid_planner/";  // 参数命名空间

    // 用nh.param()简洁加载参数，格式统一：(参数名, 变量, 默认值)
    nh_.param(ns + "att_gain", att_gain_, 1.0);
    nh_.param(ns + "rep_gain", rep_gain_, 10.0);
    nh_.param(ns + "rep_radius", rep_radius_, 2.0);
    nh_.param(ns + "max_force", max_force_, 50.0);
    nh_.param(ns + "field_resolution", field_resolution_, 0.5);
    nh_.param(ns + "field_range", field_range_, 10.0);
    nh_.param(ns + "attract_range", attract_range_, 50.0);
}
}