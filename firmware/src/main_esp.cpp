#include <Arduino.h>
#include <ArduinoJson.h> // ใช้ ArduinoJson แทนการแกะ String เอง

#include "initial.h"
#include "imu.h"
#include "ultrasonic.h"
#include "motor.h"
#include "vibration.h"
#include "pid.h"

// ===================== SERIAL CONFIG =====================
static constexpr uint32_t JETSON_BAUD = 115200;
static constexpr uint32_t ENC_BAUD    = 115200;
static constexpr int ENC_UART_RX_PIN  = 16;  
static constexpr int ENC_UART_TX_PIN  = 17;  
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

static float V_MAX_MPS[4] = { 1.33f, 1.33f, 5.80f, 5.80f };

static int ENC_SIGN[4]   = { +1, +1, +1, +1 };
static int MOTOR_SIGN[4] = { +1, +1, +1, +1 };
static constexpr float ENC_VEL_LPF_ALPHA = 0.35f;

static float PID_KP[4] = { 0.25f, 0.25f, 0.20f, 0.20f };
static float PID_KI[4] = { 0.08f, 0.08f, 0.05f, 0.05f };
static float PID_KD[4] = { 0.00f, 0.00f, 0.00f, 0.00f };

static constexpr uint32_t TELEMETRY_PERIOD_MS = 50;   
static bool DEBUG_TO_USB = false;
static uint32_t DEBUG_PERIOD_MS = 200;
// =========================================================


// ===================== state =====================
static PID pid[4];

static float v_cmd[4]        = {0,0,0,0};   
static float enc_vel_raw[4]  = {0,0,0,0};   
static float enc_vel_corr[4] = {0,0,0,0};   
static int64_t wheel_cnt[4]  = {0,0,0,0};

static float u_ff[4]   = {0,0,0,0};
static float u_pid[4]  = {0,0,0,0};
static float u_out[4]  = {0,0,0,0};
static float u_send[4] = {0,0,0,0};

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
  if (!isfinite(x)) return 0.0f;
  
  // เพิ่มบรรทัดนี้: ถ้าค่ามันเล็กจิ๋วมากๆ (น้อยกว่า 0.0001) ให้ปัดเป็น 0.0 ไปเลย
  if (fabsf(x) < 0.0001f) return 0.0f; 
  
  return x;
}

// ใช้งาน LineReader เหมือนเดิม เพราะเป็นวิธีรับข้อมูล UART ที่ Non-blocking ดีที่สุด
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
        n = 0; 
      }
    }
    return false;
  }
};

static LineReader lr_usb;
static LineReader lr_enc;


static void sendBuzzerDurationToEnc(float duration_sec) {
  if (!isfinite(duration_sec)) return;
  if (duration_sec < 0.0f) duration_sec = 0.0f;
  Serial2.printf("B:%.3f\n", duration_sec);
}

static void stopAll() {
  for (int i = 0; i < 4; ++i) {
    v_cmd[i] = 0.0f; u_ff[i] = 0.0f; u_pid[i] = 0.0f; u_out[i] = 0.0f; u_send[i] = 0.0f;
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

// ===================== JSON Parsers =====================

static void handleJetsonLine(const char *line) {
  // เช็ค Text Command พื้นฐานก่อน
  if (handleDebugTextCmd(line)) return;

  // แปลง JSON
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) return;

  if (doc.containsKey("wheel_cmd")) {
    JsonArray arr = doc["wheel_cmd"];
    for (int i = 0; i < 4; i++) {
      float val = arr[i];
      v_cmd[i] = (fabsf(val) < V_CMD_DEADBAND) ? 0.0f : val;
    }
    last_cmd_ms = millis();
  }

  if (doc.containsKey("buzzer_duration")) {
    sendBuzzerDurationToEnc(doc["buzzer_duration"]);
  }

  if (doc.containsKey("vibration_enable")) {
    set_vibration(doc["vibration_enable"]);
  }

  if (doc.containsKey("dbg_enable")) {
    DEBUG_TO_USB = doc["dbg_enable"];
  }

  if (doc.containsKey("dbg_rate_ms")) {
    uint32_t ms = doc["dbg_rate_ms"];
    if (ms < 50) ms = 50;
    if (ms > 5000) ms = 5000;
    DEBUG_PERIOD_MS = ms;
  }
}

static void handleEncLine(const char *line) {
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) return;

  bool ok_any = false;

  if (doc.containsKey("enc_vel") || doc.containsKey("enc_vel_mps")) {
    JsonArray arr = doc.containsKey("enc_vel") ? doc["enc_vel"] : doc["enc_vel_mps"];
    for (int i = 0; i < 4; i++) enc_vel_raw[i] = arr[i];
    ok_any = true;
  }

  if (doc.containsKey("wheel_cnt") || doc.containsKey("enc_counts")) {
    JsonArray arr = doc.containsKey("wheel_cnt") ? doc["wheel_cnt"] : doc["enc_counts"];
    for (int i = 0; i < 4; i++) wheel_cnt[i] = arr[i];
    ok_any = true;
  }

  if (ok_any) last_enc_ms = millis();
}

// ===================== JSON Publishers =====================

static void publishTelemetry() {
  ImuData imu_vals = get_imu_data();
  UltrasonicData us = get_ultrasonic_data();

  StaticJsonDocument<512> doc;

  JsonArray enc_arr = doc.createNestedArray("enc_vel");
  for(int i=0; i<4; i++) enc_arr.add(safeFloat(enc_vel_corr[i]));

  JsonArray quat_arr = doc.createNestedArray("imu_quat");
  quat_arr.add(safeFloat(imu_vals.qx)); quat_arr.add(safeFloat(imu_vals.qy)); 
  quat_arr.add(safeFloat(imu_vals.qz)); quat_arr.add(safeFloat(imu_vals.qw));

  JsonArray gyro_arr = doc.createNestedArray("imu_gyro");
  gyro_arr.add(safeFloat(imu_vals.gx)); gyro_arr.add(safeFloat(imu_vals.gy)); gyro_arr.add(safeFloat(imu_vals.gz));

  JsonArray acc_arr = doc.createNestedArray("imu_lin_acc");
  acc_arr.add(safeFloat(imu_vals.ax)); acc_arr.add(safeFloat(imu_vals.ay)); acc_arr.add(safeFloat(imu_vals.az));

  JsonArray us_arr = doc.createNestedArray("ultrasonic");
  us_arr.add(safeFloat(us.left)); us_arr.add(safeFloat(us.middle)); us_arr.add(safeFloat(us.right));

  serializeJson(doc, Serial);
  Serial.println();
}

static void publishDebug() {
  const uint32_t now_ms = millis();
  const ImuData imu_vals = get_imu_data();
  
  StaticJsonDocument<1024> doc;
  JsonObject dbg = doc.createNestedObject("debug");

  JsonArray cmd_arr = dbg.createNestedArray("wheel_cmd");
  for(int i=0; i<4; i++) cmd_arr.add(safeFloat(v_cmd[i]));

  JsonArray raw_arr = dbg.createNestedArray("enc_vel_raw");
  for(int i=0; i<4; i++) raw_arr.add(safeFloat(enc_vel_raw[i]));

  JsonArray corr_arr = dbg.createNestedArray("enc_vel_corr");
  for(int i=0; i<4; i++) corr_arr.add(safeFloat(enc_vel_corr[i]));

  JsonArray cnt_arr = dbg.createNestedArray("wheel_cnt");
  for(int i=0; i<4; i++) cnt_arr.add((long long)wheel_cnt[i]);

  JsonArray ff_arr = dbg.createNestedArray("u_ff");
  for(int i=0; i<4; i++) ff_arr.add(safeFloat(u_ff[i]));

  JsonArray pid_arr = dbg.createNestedArray("u_pid");
  for(int i=0; i<4; i++) pid_arr.add(safeFloat(u_pid[i]));

  JsonArray out_arr = dbg.createNestedArray("motor_u");
  for(int i=0; i<4; i++) out_arr.add(safeFloat(u_send[i]));

  dbg["cmd_age_ms"] = now_ms - last_cmd_ms;
  dbg["enc_age_ms"] = now_ms - last_enc_ms;
  dbg["imu_ok"] = imu_is_ready() ? 1 : 0;
  dbg["vibration_enable"] = get_vibration_state() ? 1 : 0;

  JsonArray q_arr = dbg.createNestedArray("imu_quat");
  q_arr.add(safeFloat(imu_vals.qx)); q_arr.add(safeFloat(imu_vals.qy)); 
  q_arr.add(safeFloat(imu_vals.qz)); q_arr.add(safeFloat(imu_vals.qw));

  serializeJson(doc, Serial);
  Serial.println();
}

// ===================== Main Setup & Loop =====================

void setup() {
  Serial.begin(JETSON_BAUD);
  Serial2.begin(ENC_BAUD, SERIAL_8N1, ENC_UART_RX_PIN, ENC_UART_TX_PIN);

  Serial.setRxBufferSize(1024);
  Serial2.setRxBufferSize(2048);

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
  static char line_cmd[256];
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
          u_ff[i] = 0.0f; u_pid[i] = 0.0f; u_out[i] = 0.0f; u_send[i] = 0.0f;
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