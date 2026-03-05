#include <Arduino.h>
#include <string.h>

#include "initial.h"
#include "imu.h"
#include "ultrasonic.h"
#include "motor.h"
#include "pid.h"

// ===================== SERIAL CONFIG =====================
static constexpr uint32_t JETSON_BAUD = 115200;      // USB Serial from Jetson
static constexpr uint32_t ENC_BAUD    = 115200;      // UART2 from ESP32-ENC
static constexpr int ENC_UART_RX_PIN  = 16;          // MAIN RX2 (to ENC_TX)
static constexpr int ENC_UART_TX_PIN  = 17;          // MAIN TX2 (to ENC_RX)
// =========================================================

// ===================== CONTROL CONFIG =====================
static constexpr float CONTROL_HZ = 100.0f;

// แนะนำตอนเทส: กัน cmd ยิงไม่ถี่พอ
static constexpr uint32_t CMD_TIMEOUT_MS = 1000;
static constexpr uint32_t ENC_TIMEOUT_MS = 300;

// enable wheels
static constexpr bool ENABLE_WHEEL[4] = { true, true, true, true };

// ตอนนี้ encoder ใช้ได้ 4 ล้อแล้ว
static constexpr bool USE_CLOSED_LOOP[4] = { true, true, true, true };

// deadband ของ setpoint (m/s) กันสั่นใกล้ศูนย์
static constexpr float V_CMD_DEADBAND = 0.005f;

// limit output to motor [-1..1]
static constexpr float U_MIN = -1.0f;
static constexpr float U_MAX =  1.0f;

// feedforward แบบ V_MAX ต่อ wheel: u_ff = v_cmd / V_MAX
// ต้องคาลิเบรตจากการทดลอง
static float V_MAX_MPS[4] = {
  1.33f, // FL
  1.33f, // FR
  5.8f, // RL
  5.8f  // RR
};

// sign correction
// เป้าหมาย: v_cmd บวก -> enc_vel_corr ต้องบวก
static int ENC_SIGN[4]   = { +1, +1, +1, +1 };  // ถ้า enc_vel กลับทิศ ให้เปลี่ยนเป็น -1 เฉพาะล้อนั้น
static int MOTOR_SIGN[4] = { +1, +1, +1, +1 };  // ถ้ามอเตอร์กลับทิศ ให้เปลี่ยนเป็น -1 เฉพาะล้อนั้น

// simple low-pass for enc_vel on MAIN (เพิ่มความนิ่ง)
static constexpr float ENC_VEL_LPF_ALPHA = 0.35f;

// debug toggle (สำคัญ: ป้องกัน Jetson ไม่อ่านแล้วบล็อก)
static bool DEBUG_TO_USB = false;
static uint32_t DEBUG_PERIOD_MS = 200;
// ===========================================================

// ===================== state =====================
static PID pid[4];

static float v_cmd[4]        = {0,0,0,0};     // from jetson (m/s)
static float enc_vel_raw[4]  = {0,0,0,0};     // from enc board
static float enc_vel_corr[4] = {0,0,0,0};     // after sign + lpf
static int64_t wheel_cnt[4]  = {0,0,0,0};

static float u_ff[4]   = {0,0,0,0};
static float u_pid[4]  = {0,0,0,0};
static float u_out[4]  = {0,0,0,0};           // before MOTOR_SIGN
static float u_send[4] = {0,0,0,0};           // after MOTOR_SIGN -> motor.cpp

static uint32_t last_cmd_ms  = 0;
static uint32_t last_enc_ms  = 0;
static uint32_t last_ctrl_us = 0;
static uint32_t last_pub_ms  = 0;

// ---------------- helpers ----------------
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}
static inline float safeFloat(float x) {
  return isfinite(x) ? x : 0.0f;
}

struct LineReader {
  size_t n = 0;
  bool read(Stream &s, char *buf, size_t cap) {
    while (s.available()) {
      char c = (char)s.read();
      if (c == '\r') continue;
      if (c == '\n') { buf[n] = '\0'; n = 0; return true; }
      if (n + 1 < cap) buf[n++] = c;
      else n = 0; // overflow -> reset
    }
    return false;
  }
};
static LineReader lr_usb;
static LineReader lr_enc;

static bool parseArray4f(const char *line, const char *key, float out[4]) {
  const char *k = strstr(line, key);
  if (!k) return false;
  const char *lb = strchr(k, '[');
  if (!lb) return false;

  char *endp = nullptr;
  const char *p = lb + 1;

  for (int i = 0; i < 4; i++) {
    out[i] = strtof(p, &endp);
    if (endp == p) return false;
    p = endp;

    const char sep = (i < 3) ? ',' : ']';
    const char *s = strchr(p, sep);
    if (!s) return false;
    p = s + 1;
  }
  return true;
}

static bool parseArray4i64(const char *line, const char *key, int64_t out[4]) {
  const char *k = strstr(line, key);
  if (!k) return false;
  const char *lb = strchr(k, '[');
  if (!lb) return false;

  char *endp = nullptr;
  const char *p = lb + 1;

  for (int i = 0; i < 4; i++) {
    long long v = strtoll(p, &endp, 10);
    if (endp == p) return false;
    out[i] = (int64_t)v;
    p = endp;

    const char sep = (i < 3) ? ',' : ']';
    const char *s = strchr(p, sep);
    if (!s) return false;
    p = s + 1;
  }
  return true;
}

static void stopAll() {
  for (int i = 0; i < 4; i++) {
    v_cmd[i] = 0.0f;
    u_ff[i] = u_pid[i] = u_out[i] = u_send[i] = 0.0f;
    pid[i].reset();
  }
  set_motor_speeds(0,0,0,0);
}

// ---- debug command from Jetson ----
// "DBG 1" / "DBG 0"
// "DBGRATE 200" (ms)
static bool handleDebugCmd(const char *line) {
  if (strncmp(line, "DBG", 3) == 0) {
    // allow: "DBG 1" / "DBG 0"
    const char *p = line + 3;
    while (*p == ' ') p++;
    int v = atoi(p);
    DEBUG_TO_USB = (v != 0);
    Serial.printf("{\"dbg\":%d}\n", DEBUG_TO_USB ? 1 : 0);
    return true;
  }
  if (strncmp(line, "DBGRATE", 7) == 0) {
    const char *p = line + 7;
    while (*p == ' ') p++;
    int ms = atoi(p);
    if (ms < 50) ms = 50;
    if (ms > 2000) ms = 2000;
    DEBUG_PERIOD_MS = (uint32_t)ms;
    Serial.printf("{\"dbg_rate_ms\":%lu}\n", (unsigned long)DEBUG_PERIOD_MS);
    return true;
  }
  return false;
}

// ---------------- setup ----------------
void setup() {
  system_begin();
  delay(200);

  // เพิ่ม buffer กัน overflow ตอนพิมพ์/รับถี่
  Serial.setRxBufferSize(1024);
  Serial2.setRxBufferSize(2048);

  Serial.begin(JETSON_BAUD);
  Serial2.begin(ENC_BAUD, SERIAL_8N1, ENC_UART_RX_PIN, ENC_UART_TX_PIN);

  // PID init (เริ่ม conservative)
  for (int i = 0; i < 4; i++) {
    pid[i].setGains(0.0f, 0.0f, 0.00f);
    pid[i].setLimits(U_MIN, U_MAX, -0.5f, 0.5f);
    pid[i].reset();
  }

  last_cmd_ms = millis();
  last_enc_ms = millis();
  last_ctrl_us = micros();
  last_pub_ms  = millis();

  // แจ้งการใช้งาน
  Serial.println("{\"boot\":\"ok\",\"hint\":\"send wheel_cmd JSON or DBG/DBGRATE\"}");
}

// ---------------- main loop ----------------
void loop() {
  // 1) RX from Jetson (USB)
  static char line_cmd[128];
  if (lr_usb.read(Serial, line_cmd, sizeof(line_cmd))) {
    if (!handleDebugCmd(line_cmd)) {
      float tmp[4];
      if (parseArray4f(line_cmd, "wheel_cmd", tmp)) {
        for (int i = 0; i < 4; i++) {
          v_cmd[i] = (fabsf(tmp[i]) < V_CMD_DEADBAND) ? 0.0f : tmp[i];
        }
        last_cmd_ms = millis();
      }
    }
  }

  // 2) RX encoder from ESP32-ENC (UART2)
  static char line_enc[256];
  if (lr_enc.read(Serial2, line_enc, sizeof(line_enc))) {
    bool ok_any = false;

    float tmpv[4];
    if (parseArray4f(line_enc, "enc_vel", tmpv) || parseArray4f(line_enc, "enc_vel_mps", tmpv)) {
      for (int i = 0; i < 4; i++) enc_vel_raw[i] = tmpv[i];
      ok_any = true;
    }

    int64_t tmpc[4];
    if (parseArray4i64(line_enc, "wheel_cnt", tmpc) || parseArray4i64(line_enc, "enc_counts", tmpc)) {
      for (int i = 0; i < 4; i++) wheel_cnt[i] = tmpc[i];
      ok_any = true;
    }

    if (ok_any) last_enc_ms = millis();
  }

  // 3) corrected enc vel (sign + lpf)
  for (int i = 0; i < 4; i++) {
    const float v = (float)ENC_SIGN[i] * enc_vel_raw[i];
    enc_vel_corr[i] = ENC_VEL_LPF_ALPHA * v + (1.0f - ENC_VEL_LPF_ALPHA) * enc_vel_corr[i];
  }

  // 4) Control loop @ CONTROL_HZ
  const uint32_t now_us = micros();
  const float dt = (now_us - last_ctrl_us) / 1e6f;

  if (dt >= (1.0f / CONTROL_HZ)) {
    last_ctrl_us = now_us;

    const uint32_t now_ms = millis();
    const uint32_t cmd_age = now_ms - last_cmd_ms;
    const uint32_t enc_age = now_ms - last_enc_ms;

    const bool cmd_ok = (cmd_age <= CMD_TIMEOUT_MS);
    const bool enc_ok = (enc_age <= ENC_TIMEOUT_MS);

    if (!cmd_ok || !enc_ok) {
      stopAll();
    } else {
      for (int i = 0; i < 4; i++) {
        if (!ENABLE_WHEEL[i]) {
          u_ff[i] = u_pid[i] = u_out[i] = u_send[i] = 0.0f;
          pid[i].reset();
          continue;
        }

        // ✅ feedforward แบบ V_MAX
        const float vmax = (V_MAX_MPS[i] > 1e-3f) ? V_MAX_MPS[i] : 1.0f;
        u_ff[i] = clampf(v_cmd[i] / vmax, U_MIN, U_MAX);

        if (v_cmd[i] == 0.0f) {
          u_pid[i] = 0.0f;
          u_out[i] = 0.0f;
          pid[i].reset();
        } else if (USE_CLOSED_LOOP[i]) {
          u_pid[i] = pid[i].update(v_cmd[i], enc_vel_corr[i], dt);
          u_out[i] = clampf(u_ff[i] + u_pid[i], U_MIN, U_MAX);
        } else {
          u_pid[i] = 0.0f;
          u_out[i] = u_ff[i];
          pid[i].reset();
        }

        u_send[i] = (float)MOTOR_SIGN[i] * u_out[i];
      }

      set_motor_speeds(u_send[0], u_send[1], u_send[2], u_send[3]);
    }
  }

  // 5) Debug print (ถ้า Jetson ไม่ read ให้ปิด DBG)
  const uint32_t now_ms = millis();
  if (DEBUG_TO_USB && (now_ms - last_pub_ms >= DEBUG_PERIOD_MS)) {
    last_pub_ms = now_ms;

    ImuData imu_vals = get_imu_data();
    UltrasonicData ultrasonic_vals = get_ultrasonic_data();

    const uint32_t enc_age = now_ms - last_enc_ms;
    const uint32_t cmd_age = now_ms - last_cmd_ms;

    Serial.printf(
      "{\"wheel_cmd\":[%.3f,%.3f,%.3f,%.3f],"
      "\"enc_vel\":[%.3f,%.3f,%.3f,%.3f],"
      "\"wheel_cnt\":[%lld,%lld,%lld,%lld],"
      "\"u_ff\":[%.3f,%.3f,%.3f,%.3f],"
      "\"u_pid\":[%.3f,%.3f,%.3f,%.3f],"
      "\"motor_u\":[%.3f,%.3f,%.3f,%.3f],"
      "\"enc_age_ms\":%lu,\"cmd_age_ms\":%lu,"
      "\"ultrasonic\":[%.2f,%.2f,%.2f],"
      "\"imu_gyro\":[%.2f,%.2f,%.2f]}\n",

      safeFloat(v_cmd[0]), safeFloat(v_cmd[1]), safeFloat(v_cmd[2]), safeFloat(v_cmd[3]),
      safeFloat(enc_vel_corr[0]), safeFloat(enc_vel_corr[1]), safeFloat(enc_vel_corr[2]), safeFloat(enc_vel_corr[3]),
      (long long)wheel_cnt[0], (long long)wheel_cnt[1], (long long)wheel_cnt[2], (long long)wheel_cnt[3],
      safeFloat(u_ff[0]), safeFloat(u_ff[1]), safeFloat(u_ff[2]), safeFloat(u_ff[3]),
      safeFloat(u_pid[0]), safeFloat(u_pid[1]), safeFloat(u_pid[2]), safeFloat(u_pid[3]),
      safeFloat(u_send[0]), safeFloat(u_send[1]), safeFloat(u_send[2]), safeFloat(u_send[3]),
      (unsigned long)enc_age, (unsigned long)cmd_age,

      safeFloat(ultrasonic_vals.left), safeFloat(ultrasonic_vals.middle), safeFloat(ultrasonic_vals.right),
      safeFloat(imu_vals.gx), safeFloat(imu_vals.gy), safeFloat(imu_vals.gz)
    );
  }

  Serial.printf("{\"enc_vel\":[%.3f,%.3f,%.3f,%.3f]}", safeFloat(enc_vel_corr[0]), safeFloat(enc_vel_corr[1]), safeFloat(enc_vel_corr[2]), safeFloat(enc_vel_corr[3]));
}