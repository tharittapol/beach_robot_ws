#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

#include "initial.h"
#include "imu.h"
#include "ultrasonic.h"
#include "motor.h"
#include "pid.h"

// ===================== SERIAL CONFIG =====================
static constexpr uint32_t JETSON_BAUD = 115200;
static constexpr uint32_t ENC_BAUD    = 115200;
static constexpr int ENC_UART_RX_PIN  = 16;  // MAIN RX2 <- ENC TX
static constexpr int ENC_UART_TX_PIN  = 17;  // MAIN TX2 -> ENC RX
// =========================================================

// ===================== CONTROL CONFIG =====================
static constexpr float CONTROL_HZ = 100.0f;
static constexpr uint32_t CMD_TIMEOUT_MS = 1000;
static constexpr uint32_t ENC_TIMEOUT_MS = 300;

static constexpr bool ENABLE_WHEEL[4]    = { true, true, true, true };
static constexpr bool USE_CLOSED_LOOP[4] = { true, true, true, true };

static constexpr float V_CMD_DEADBAND = 0.005f;
static constexpr float U_MIN = -1.0f;
static constexpr float U_MAX =  1.0f;

// feedforward scale per wheel (u_ff = v_cmd / V_MAX_MPS[i])
static float V_MAX_MPS[4] = {
  1.33f, 1.33f, 5.80f, 5.80f
};

// sign correction
static int ENC_SIGN[4]   = { +1, +1, +1, +1 };
static int MOTOR_SIGN[4] = { +1, +1, +1, +1 };

// encoder velocity LPF on MAIN
static constexpr float ENC_VEL_LPF_ALPHA = 0.35f;

// initial PID values = safe starting point only, must tune on real robot
static float PID_KP[4] = { 0.25f, 0.25f, 0.20f, 0.20f };
static float PID_KI[4] = { 0.08f, 0.08f, 0.05f, 0.05f };
static float PID_KD[4] = { 0.00f, 0.00f, 0.00f, 0.00f };

// telemetry / debug
static constexpr uint32_t TELEMETRY_PERIOD_MS = 50;   // 20 Hz production stream
static bool DEBUG_TO_USB = false;
static uint32_t DEBUG_PERIOD_MS = 200;
// =========================================================

// ===================== AUX OUTPUT CONFIG =====================
static constexpr int VIBRATION_PIN = 4;   // <<< เปลี่ยนตามขาจริงบนบอร์ด
// =============================================================

// ===================== state =====================
static PID pid[4];

static float v_cmd[4]        = {0,0,0,0};   // m/s command from Jetson
static float enc_vel_raw[4]  = {0,0,0,0};   // raw from ENC board
static float enc_vel_corr[4] = {0,0,0,0};   // sign-corrected + filtered
static int64_t wheel_cnt[4]  = {0,0,0,0};

static float u_ff[4]   = {0,0,0,0};
static float u_pid[4]  = {0,0,0,0};
static float u_out[4]  = {0,0,0,0};
static float u_send[4] = {0,0,0,0};

static bool vibration_enable = false;

static uint32_t last_cmd_ms       = 0;
static uint32_t last_enc_ms       = 0;
static uint32_t last_ctrl_us      = 0;
static uint32_t last_telemetry_ms = 0;
static uint32_t last_debug_ms     = 0;

// ===================== helpers =====================
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
      if (c == '\n') {
        buf[n] = '\0';
        n = 0;
        return true;
      }
      if (n + 1 < cap) {
        buf[n++] = c;
      } else {
        n = 0;  // overflow -> wait for next full line
      }
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

  const char *p = lb + 1;
  char *endp = nullptr;

  for (int i = 0; i < 4; ++i) {
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

  const char *p = lb + 1;
  char *endp = nullptr;

  for (int i = 0; i < 4; ++i) {
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

static bool parseFloatKey(const char *line, const char *key, float &out) {
  const char *k = strstr(line, key);
  if (!k) return false;

  const char *c = strchr(k, ':');
  if (!c) return false;
  ++c;

  while (*c == ' ' || *c == '\t') ++c;

  char *endp = nullptr;
  out = strtof(c, &endp);
  return endp != c;
}

static bool parseUIntKey(const char *line, const char *key, uint32_t &out) {
  float tmp = 0.0f;
  if (!parseFloatKey(line, key, tmp)) return false;
  if (tmp < 0.0f) tmp = 0.0f;
  out = (uint32_t)tmp;
  return true;
}

static bool parseBoolKey(const char *line, const char *key, bool &out) {
  const char *k = strstr(line, key);
  if (!k) return false;

  const char *c = strchr(k, ':');
  if (!c) return false;
  ++c;

  while (*c == ' ' || *c == '\t') ++c;

  if (strncmp(c, "true", 4) == 0)  { out = true;  return true; }
  if (strncmp(c, "false", 5) == 0) { out = false; return true; }
  if (*c == '1') { out = true;  return true; }
  if (*c == '0') { out = false; return true; }
  return false;
}

static void applyVibration(bool enable) {
  vibration_enable = enable;
  digitalWrite(VIBRATION_PIN, vibration_enable ? HIGH : LOW);
}

static void sendBuzzerDurationToEnc(float duration_sec) {
  if (!isfinite(duration_sec)) return;
  if (duration_sec < 0.0f) duration_sec = 0.0f;
  Serial2.printf("B:%.3f\n", duration_sec);
}

static void stopAll() {
  for (int i = 0; i < 4; ++i) {
    v_cmd[i] = 0.0f;
    u_ff[i] = 0.0f;
    u_pid[i] = 0.0f;
    u_out[i] = 0.0f;
    u_send[i] = 0.0f;
    pid[i].reset();
  }
  set_motor_speeds(0.0f, 0.0f, 0.0f, 0.0f);
}

static bool handleDebugTextCmd(const char *line) {
  if (strncmp(line, "DBG", 3) == 0) {
    const char *p = line + 3;
    while (*p == ' ') ++p;
    DEBUG_TO_USB = (atoi(p) != 0);
    return true;
  }

  if (strncmp(line, "DBGRATE", 7) == 0) {
    const char *p = line + 7;
    while (*p == ' ') ++p;
    int ms = atoi(p);
    if (ms < 50) ms = 50;
    if (ms > 5000) ms = 5000;
    DEBUG_PERIOD_MS = (uint32_t)ms;
    return true;
  }

  return false;
}

static void handleJetsonLine(const char *line) {
  if (handleDebugTextCmd(line)) return;

  float cmd4[4];
  if (parseArray4f(line, "wheel_cmd", cmd4)) {
    for (int i = 0; i < 4; ++i) {
      v_cmd[i] = (fabsf(cmd4[i]) < V_CMD_DEADBAND) ? 0.0f : cmd4[i];
    }
    last_cmd_ms = millis();
  }

  float buzzer_duration = 0.0f;
  if (parseFloatKey(line, "buzzer_duration", buzzer_duration)) {
    sendBuzzerDurationToEnc(buzzer_duration);
  }

  bool vib = false;
  if (parseBoolKey(line, "vibration_enable", vib)) {
    applyVibration(vib);
  }

  bool dbg = false;
  if (parseBoolKey(line, "dbg_enable", dbg)) {
    DEBUG_TO_USB = dbg;
  }

  uint32_t dbg_rate = 0;
  if (parseUIntKey(line, "dbg_rate_ms", dbg_rate)) {
    if (dbg_rate < 50) dbg_rate = 50;
    if (dbg_rate > 5000) dbg_rate = 5000;
    DEBUG_PERIOD_MS = dbg_rate;
  }
}

static void handleEncLine(const char *line) {
  bool ok_any = false;

  float tmpv[4];
  if (parseArray4f(line, "enc_vel", tmpv) || parseArray4f(line, "enc_vel_mps", tmpv)) {
    for (int i = 0; i < 4; ++i) {
      enc_vel_raw[i] = tmpv[i];
    }
    ok_any = true;
  }

  int64_t tmpc[4];
  if (parseArray4i64(line, "wheel_cnt", tmpc) || parseArray4i64(line, "enc_counts", tmpc)) {
    for (int i = 0; i < 4; ++i) {
      wheel_cnt[i] = tmpc[i];
    }
    ok_any = true;
  }

  if (ok_any) {
    last_enc_ms = millis();
  }
}

static void publishTelemetry() {
  ImuData imu_vals = get_imu_data();
  UltrasonicData us = get_ultrasonic_data();

  Serial.printf(
    "{\"enc_vel\":[%.6f,%.6f,%.6f,%.6f],"
    "\"imu_quat\":[%.6f,%.6f,%.6f,%.6f],"
    "\"imu_gyro\":[%.6f,%.6f,%.6f],"
    "\"imu_lin_acc\":[%.6f,%.6f,%.6f],"
    "\"ultrasonic\":[%.3f,%.3f,%.3f]}\n",
    safeFloat(enc_vel_corr[0]), safeFloat(enc_vel_corr[1]), safeFloat(enc_vel_corr[2]), safeFloat(enc_vel_corr[3]),
    safeFloat(imu_vals.qx), safeFloat(imu_vals.qy), safeFloat(imu_vals.qz), safeFloat(imu_vals.qw),
    safeFloat(imu_vals.gx), safeFloat(imu_vals.gy), safeFloat(imu_vals.gz),
    safeFloat(imu_vals.ax), safeFloat(imu_vals.ay), safeFloat(imu_vals.az),
    safeFloat(us.left), safeFloat(us.middle), safeFloat(us.right)
  );
}

static void publishDebug() {
  const uint32_t now_ms = millis();
  const uint32_t cmd_age = now_ms - last_cmd_ms;
  const uint32_t enc_age = now_ms - last_enc_ms;
  const ImuData imu_vals = get_imu_data();
  const bool imu_ok = imu_is_ready();

  Serial.printf(
    "{\"debug\":{"
    "\"wheel_cmd\":[%.4f,%.4f,%.4f,%.4f],"
    "\"enc_vel_raw\":[%.4f,%.4f,%.4f,%.4f],"
    "\"enc_vel_corr\":[%.4f,%.4f,%.4f,%.4f],"
    "\"wheel_cnt\":[%lld,%lld,%lld,%lld],"
    "\"u_ff\":[%.4f,%.4f,%.4f,%.4f],"
    "\"u_pid\":[%.4f,%.4f,%.4f,%.4f],"
    "\"motor_u\":[%.4f,%.4f,%.4f,%.4f],"
    "\"cmd_age_ms\":%lu,"
    "\"enc_age_ms\":%lu,"
    "\"imu_ok\":%d,"
    "\"vibration_enable\":%d,"
    "\"imu_quat\":[%.6f,%.6f,%.6f,%.6f]"
    "}}\n",
    safeFloat(v_cmd[0]), safeFloat(v_cmd[1]), safeFloat(v_cmd[2]), safeFloat(v_cmd[3]),
    safeFloat(enc_vel_raw[0]), safeFloat(enc_vel_raw[1]), safeFloat(enc_vel_raw[2]), safeFloat(enc_vel_raw[3]),
    safeFloat(enc_vel_corr[0]), safeFloat(enc_vel_corr[1]), safeFloat(enc_vel_corr[2]), safeFloat(enc_vel_corr[3]),
    (long long)wheel_cnt[0], (long long)wheel_cnt[1], (long long)wheel_cnt[2], (long long)wheel_cnt[3],
    safeFloat(u_ff[0]), safeFloat(u_ff[1]), safeFloat(u_ff[2]), safeFloat(u_ff[3]),
    safeFloat(u_pid[0]), safeFloat(u_pid[1]), safeFloat(u_pid[2]), safeFloat(u_pid[3]),
    safeFloat(u_send[0]), safeFloat(u_send[1]), safeFloat(u_send[2]), safeFloat(u_send[3]),
    (unsigned long)cmd_age,
    (unsigned long)enc_age,
    imu_ok ? 1 : 0,
    vibration_enable ? 1 : 0,
    safeFloat(imu_vals.qx), safeFloat(imu_vals.qy), safeFloat(imu_vals.qz), safeFloat(imu_vals.qw)
  );
}

void setup() {
  Serial.begin(JETSON_BAUD);
  Serial2.begin(ENC_BAUD, SERIAL_8N1, ENC_UART_RX_PIN, ENC_UART_TX_PIN);

  Serial.setRxBufferSize(1024);
  Serial2.setRxBufferSize(2048);

  pinMode(VIBRATION_PIN, OUTPUT);
  applyVibration(false);

  system_begin();

  for (int i = 0; i < 4; ++i) {
    pid[i].setGains(PID_KP[i], PID_KI[i], PID_KD[i]);
    pid[i].setLimits(U_MIN, U_MAX, -0.5f, 0.5f);
    pid[i].reset();
  }

  last_cmd_ms = millis();
  last_enc_ms = millis();
  last_ctrl_us = micros();
  last_telemetry_ms = millis();
  last_debug_ms = millis();
}

void loop() {
  static char line_cmd[192];
  if (lr_usb.read(Serial, line_cmd, sizeof(line_cmd))) {
    handleJetsonLine(line_cmd);
  }

  static char line_enc[256];
  if (lr_enc.read(Serial2, line_enc, sizeof(line_enc))) {
    handleEncLine(line_enc);
  }

  imu_poll();
  ultrasonic_poll();

  for (int i = 0; i < 4; ++i) {
    const float v = (float)ENC_SIGN[i] * enc_vel_raw[i];
    enc_vel_corr[i] = ENC_VEL_LPF_ALPHA * v + (1.0f - ENC_VEL_LPF_ALPHA) * enc_vel_corr[i];
  }

  const uint32_t now_us = micros();
  const float dt = (now_us - last_ctrl_us) / 1000000.0f;

  if (dt >= (1.0f / CONTROL_HZ)) {
    last_ctrl_us = now_us;

    const uint32_t now_ms = millis();
    const bool cmd_ok = ((now_ms - last_cmd_ms) <= CMD_TIMEOUT_MS);
    const bool enc_ok = ((now_ms - last_enc_ms) <= ENC_TIMEOUT_MS);

    if (!cmd_ok || !enc_ok) {
      stopAll();
    } else {
      for (int i = 0; i < 4; ++i) {
        if (!ENABLE_WHEEL[i]) {
          u_ff[i] = 0.0f;
          u_pid[i] = 0.0f;
          u_out[i] = 0.0f;
          u_send[i] = 0.0f;
          pid[i].reset();
          continue;
        }

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

  const uint32_t now_ms = millis();

  if (now_ms - last_telemetry_ms >= TELEMETRY_PERIOD_MS) {
    last_telemetry_ms = now_ms;
    publishTelemetry();
  }

  if (DEBUG_TO_USB && (now_ms - last_debug_ms >= DEBUG_PERIOD_MS)) {
    last_debug_ms = now_ms;
    publishDebug();
  }
}