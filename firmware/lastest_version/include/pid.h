#pragma once
#include <Arduino.h>

class PID {
public:
  void setGains(float kp, float ki, float kd) { kp_ = kp; ki_ = ki; kd_ = kd; }

  void setLimits(float out_min, float out_max, float i_min, float i_max) {
    out_min_ = out_min; out_max_ = out_max;
    i_min_ = i_min;     i_max_ = i_max;
  }

  void reset() { i_ = 0.0f; has_prev_ = false; prev_meas_ = 0.0f; }

  float update(float set, float meas, float dt) {
    if (dt <= 0.0f) return 0.0f;

    const float err = set - meas;

    // D on measurement (ลด noise)
    float d_meas = 0.0f;
    if (has_prev_) d_meas = (meas - prev_meas_) / dt;
    prev_meas_ = meas;
    has_prev_ = true;

    // integrate (clamped)
    float i_new = i_ + ki_ * err * dt;
    if (i_new > i_max_) i_new = i_max_;
    if (i_new < i_min_) i_new = i_min_;

    float u_unsat = kp_ * err + i_new - kd_ * d_meas;

    // saturate
    float u_sat = u_unsat;
    if (u_sat > out_max_) u_sat = out_max_;
    if (u_sat < out_min_) u_sat = out_min_;

    // anti-windup: ชนเพดานแล้ว err ดันไปทิศเดียวกัน -> ไม่อัปเดต integrator
    const bool saturated = (u_sat != u_unsat);
    if (!saturated) {
      i_ = i_new;
    } else {
      const bool pushing_same_dir =
          (u_sat >= out_max_ && err > 0.0f) || (u_sat <= out_min_ && err < 0.0f);
      if (!pushing_same_dir) i_ = i_new;
    }

    return u_sat;
  }

private:
  float kp_ = 0.0f, ki_ = 0.0f, kd_ = 0.0f;
  float out_min_ = -1.0f, out_max_ = 1.0f;
  float i_min_   = -0.6f, i_max_   = 0.6f;

  float i_ = 0.0f;
  float prev_meas_ = 0.0f;
  bool has_prev_ = false;
};