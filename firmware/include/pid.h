#pragma once

class PID {
public:
    void setGains(float kp, float ki, float kd);
    void setLimits(float out_min, float out_max, float i_min, float i_max);
    void reset();
    float update(float setpoint, float measured, float dt);
private:
    float _kp, _ki, _kd;
    float _out_min, _out_max, _i_min, _i_max;
    float _integral, _prev_error;
};