#include "apoc_pkg/pidctrl.h"

// 设置目标值
void pidctrl::setSetpoint(float sp){
    setpoint = sp;
}

float pidctrl::getSetpoint() {
    return setpoint;
}