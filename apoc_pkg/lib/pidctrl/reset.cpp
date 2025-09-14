#include "apoc_pkg/pidctrl.h"

    
// 重置PID控制器
void pidctrl::reset() {
    last_error = 0.0f;
    integral = 0.0f;
    derivative = 0.0f;
    output = 0.0f;
    is_first_run = true;
}
