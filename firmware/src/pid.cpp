#include "pid.h"

void PID::setGains(float kp, float ki, float kd) {
    _kp = kp; _ki = ki; _kd = kd;
}
void PID::setLimits(float out_min, float out_max, float i_min, float i_max) {
    _out_min = out_min; _out_max = out_max; _i_min = i_min; _i_max = i_max;
}
void PID::reset() {
    _integral = 0.0f; _prev_error = 0.0f;
}
float PID::update(float setpoint, float measured, float dt) {
    // ใส่ Logic PID จริงของคุณตรงนี้
    return 0.0f; 
}