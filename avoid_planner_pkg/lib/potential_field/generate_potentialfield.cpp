#include "avoid_planner/potential_field.h"

namespace avoid_planner {

PotentialGrid PotentialFieldCalculator::generatePotentialField(){
    int el_num;
    int az_num;
    for(int el_i = 0;el_i=<el_num;i++){
        for(int az_i=0;az_i=<az_num;i++){
            calculateTotalForce(double az_i, double el_i);
            
        }
    }
}
}