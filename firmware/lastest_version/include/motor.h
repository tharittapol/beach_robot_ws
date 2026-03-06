#ifndef MOTOR_H
#define MOTOR_H

void motor_begin();

// input = normalized control u in [-1.0, +1.0], not m/s
void set_motor_speeds(float u_fl, float u_fr, float u_rl, float u_rr);

#endif