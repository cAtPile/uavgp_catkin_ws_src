/**
 * @file
 */

#include"avoid_planner_pkg/potential_field.h"

namespace avoid_planner{

void PotentialFieldCalculator::setGoal(double gaol_x,double goal_y,double goal_z){

    goal_pose_[0]=gaol_x;
    goal_pose_[1]=gaol_y;
    goal_pose_[2]=gaol_z;

}

void PotentialFieldCalculator::setCurrentPose(double current_x,double current_y,double current_z);

    current_pose_[0]=current_x;
    current_pose_[1]=current_y;
    current_pose_[2]=current_z;

}

