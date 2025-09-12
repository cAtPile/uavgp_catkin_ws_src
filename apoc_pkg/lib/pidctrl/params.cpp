#include "apoc_pkg/pidctrl.h"

// 设置PID参数
void pidctrl::setPIDctrlParams(  double k_p, double k_i, double k_d,  
                            double out_min, double out_max , 
                            double int_min, double int_max){
    kp = k_p;
    ki = k_i;
    kd = k_d;
    
    output_min = out_min;
    output_max = out_max;
    
    integral_min = int_min;
    integral_max = int_max;

}