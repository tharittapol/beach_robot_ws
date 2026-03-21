#include "motor.h"
#include <Arduino.h>

// ===== pins (ESP32 MAIN) =====
// Front-Left
#define PWM_FL 32
#define DIR_FL 33
// Front-Right
#define PWM_FR 25
#define DIR_FR 26
// Rear-Left
#define PWM_RL 27
#define DIR_RL 14
// Rear-Right
#define PWM_RR 12
#define DIR_RR 13

enum WheelIndex {
  WHEEL_FL = 0,
  WHEEL_FR = 1,
  WHEEL_RL = 2,
  WHEEL_RR = 3,
};

// ================== tuning ==================
static constexpr bool ENABLE_RR = true;

static constexpr int PWM_MAX = 255;
static constexpr float U_DEADBAND = 0.03f;

// ตั้งค่าหลัง “เทสบนพื้นจริง”
// RR needs more breakaway torque than the other wheels, so keep its minimum
// output a little higher instead of pushing Kp harder.
static int PWM_START[4] = { 40, 40, 40, 55 };  // ออกตัว
static int PWM_HOLD [4] = { 25, 25, 25, 28 };  // หมุนต่อ

static constexpr uint32_t KICK_MS = 200;      // เวลา kick ช่วยให้หลุด static friction

// ทิศทาง (u>0 จะเขียน DIR เป็นค่า forward_level)
static constexpr int FORWARD_LEVEL[4] = {
  0, // FL
  1, // FR
  0, // RL
  1  // RR
};
// ============================================

// state สำหรับ kick
static uint32_t kick_until_ms[4] = {0,0,0,0};
static bool last_nonzero[4] = {false,false,false,false};

static inline int clamp_pwm(int v) {
  if (v < 0) return 0;
  if (v > PWM_MAX) return PWM_MAX;
  return v;
}

// u in [-1..+1] -> duty 0..255 with min hold/kick
static int duty_from_u(int i, float u) {
  float a = fabsf(u);
  if (a < U_DEADBAND) return 0;

  int duty = (int)lroundf(a * PWM_MAX);
  duty = clamp_pwm(duty);

  const uint32_t now = millis();
  const bool is_kick = (now < kick_until_ms[i]);

  // ตอน kick: บังคับให้ไม่น้อยกว่า PWM_START
  if (is_kick) {
    if (duty < PWM_START[i]) duty = PWM_START[i];
  } else {
    // ตอนวิ่งปกติ: บังคับไม่น้อยกว่า PWM_HOLD
    if (duty < PWM_HOLD[i]) duty = PWM_HOLD[i];
  }

  return clamp_pwm(duty);
}

static void write_motor(int i, int pin_pwm, int pin_dir, float u) {
  int duty = duty_from_u(i, u);

  if (duty == 0) {
    analogWrite(pin_pwm, 0);
    return;
  }

  const bool forward = (u > 0.0f);
  digitalWrite(pin_dir, forward ? FORWARD_LEVEL[i] : (1 - FORWARD_LEVEL[i]));
  analogWrite(pin_pwm, duty);
}

void motor_begin() {
  pinMode(PWM_FL, OUTPUT); pinMode(DIR_FL, OUTPUT);
  pinMode(PWM_FR, OUTPUT); pinMode(DIR_FR, OUTPUT);
  pinMode(PWM_RL, OUTPUT); pinMode(DIR_RL, OUTPUT);
  pinMode(PWM_RR, OUTPUT); pinMode(DIR_RR, OUTPUT);

  set_motor_speeds(0,0,0,0);
}

void set_motor_speeds(float u_fl, float u_fr, float u_rl, float u_rr) {
  float u[4] = { u_fl, u_fr, u_rl, u_rr };

  // detect rising edge -> start kick
  const uint32_t now = millis();
  for (int i = 0; i < 4; i++) {
    const bool nz = (fabsf(u[i]) >= U_DEADBAND);
    if (nz && !last_nonzero[i]) {
      kick_until_ms[i] = now + KICK_MS;   // เริ่ม kick
    }
    last_nonzero[i] = nz;
  }

  write_motor(0, PWM_FL, DIR_FL, u_fl);
  write_motor(1, PWM_FR, DIR_FR, u_fr);
  write_motor(2, PWM_RL, DIR_RL, u_rl);

  if (ENABLE_RR) {
    write_motor(3, PWM_RR, DIR_RR, u_rr);
  } else {
    analogWrite(PWM_RR, 0);
  }
}