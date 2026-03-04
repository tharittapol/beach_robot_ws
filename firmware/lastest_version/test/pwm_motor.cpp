#include <Arduino.h>
#include <string.h>

// ================== PIN (ESP32 MAIN) ==================
#define PWM_FL 32
#define DIR_FL 33
#define PWM_FR 25
#define DIR_FR 26
#define PWM_RL 27
#define DIR_RL 14
#define PWM_RR 12   // strap pin (ระวัง)
#define DIR_RR 13
// =======================================================

// ================== UART2 from ESP32-ENC ==================
static constexpr uint32_t ENC_BAUD = 115200;
static constexpr int ENC_RX_PIN = 16; // MAIN RX2 (ต่อกับ ENC_TX)
static constexpr int ENC_TX_PIN = 17; // MAIN TX2 (ต่อกับ ENC_RX)
// ==========================================================

// ===== encoder latest =====
static float   enc_vel[4]    = {0,0,0,0};
static int64_t wheel_cnt[4]  = {0,0,0,0};
static uint32_t last_enc_ms  = 0;

// ===== streaming output =====
static bool stream_on = true;
static uint32_t stream_period_ms = 100;
static uint32_t last_stream_ms = 0;

// ---------------- Motor low-level ----------------
// ทิศทางตามโค้ดเดิมคุณ:
// FL: forward -> DIR=0
// FR: forward -> DIR=1
// RL: forward -> DIR=0
// RR: forward -> DIR=1
static inline void write_motor_raw(int pin_pwm, int pin_dir, int pwm, bool forward, int forward_level) {
  pwm = constrain(pwm, 0, 255);
  digitalWrite(pin_dir, forward ? forward_level : (1 - forward_level));
  analogWrite(pin_pwm, pwm);
}

static inline void stop_all() {
  analogWrite(PWM_FL, 0);
  analogWrite(PWM_FR, 0);
  analogWrite(PWM_RL, 0);
  analogWrite(PWM_RR, 0);
}

// ---------------- Serial2 line reader ----------------
static bool readLine(Stream &s, char *buf, size_t cap) {
  static size_t n = 0;
  while (s.available()) {
    char c = (char)s.read();
    if (c == '\r') continue;
    if (c == '\n') { buf[n] = '\0'; n = 0; return true; }
    if (n + 1 < cap) buf[n++] = c;
  }
  return false;
}

// parse {"key":[a,b,c,d]} float
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

// parse {"key":[i1,i2,i3,i4]} int64
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

// keep pumping Serial2 to update enc data
static void pump_encoder_uart2() {
  static char line_enc[256];
  if (readLine(Serial2, line_enc, sizeof(line_enc))) {
    float tmpv[4];
    int64_t tmpc[4];
    bool ok = false;

    // รองรับทั้ง enc_vel และ enc_vel_mps
    if (parseArray4f(line_enc, "enc_vel", tmpv) || parseArray4f(line_enc, "enc_vel_mps", tmpv)) {
      for (int i = 0; i < 4; i++) enc_vel[i] = tmpv[i];
      ok = true;
    }

    // รองรับทั้ง wheel_cnt และ enc_counts
    if (parseArray4i64(line_enc, "wheel_cnt", tmpc) || parseArray4i64(line_enc, "enc_counts", tmpc)) {
      for (int i = 0; i < 4; i++) wheel_cnt[i] = tmpc[i];
      ok = true;
    }

    if (ok) last_enc_ms = millis();
  }
}

static void print_enc_json() {
  Serial.printf("{\"enc_age_ms\":%lu,\"enc_vel\":[%.4f,%.4f,%.4f,%.4f],\"wheel_cnt\":[%lld,%lld,%lld,%lld]}\n",
                (unsigned long)(millis() - last_enc_ms),
                enc_vel[0], enc_vel[1], enc_vel[2], enc_vel[3],
                (long long)wheel_cnt[0], (long long)wheel_cnt[1], (long long)wheel_cnt[2], (long long)wheel_cnt[3]);
}

// ---------------- Command parser ----------------
static String line;

static void apply_one(const String& wheel, int value) {
  bool forward = (value >= 0);
  int pwm = abs(value);

  if      (wheel == "fl") write_motor_raw(PWM_FL, DIR_FL, pwm, forward, 0);
  else if (wheel == "fr") write_motor_raw(PWM_FR, DIR_FR, pwm, forward, 1);
  else if (wheel == "rl") write_motor_raw(PWM_RL, DIR_RL, pwm, forward, 0);
  else if (wheel == "rr") write_motor_raw(PWM_RR, DIR_RR, pwm, forward, 1);
  else Serial.println("Unknown wheel (use fl/fr/rl/rr)");
}

static void parse_usb_command_line() {
  line.trim();
  if (line.length() == 0) { line = ""; return; }

  if (line == "stop") {
    stop_all();
    Serial.println("STOP");
    line = "";
    return;
  }

  if (line == "show") {
    print_enc_json();
    line = "";
    return;
  }

  if (line.startsWith("stream ")) {
    String v = line.substring(7); v.trim();
    if (v == "on" || v == "1") stream_on = true;
    else if (v == "off" || v == "0") stream_on = false;
    Serial.printf("stream_on=%d\n", stream_on ? 1 : 0);
    line = "";
    return;
  }

  if (line.startsWith("rate ")) {
    String v = line.substring(5); v.trim();
    int ms = (int)v.toInt();
    if (ms < 20) ms = 20;
    if (ms > 2000) ms = 2000;
    stream_period_ms = (uint32_t)ms;
    Serial.printf("stream_period_ms=%lu\n", (unsigned long)stream_period_ms);
    line = "";
    return;
  }

  // parse: <wheel> <value>
  int sp = line.indexOf(' ');
  if (sp > 0) {
    String wheel = line.substring(0, sp); wheel.trim();
    String val_s = line.substring(sp + 1); val_s.trim();

    int value = (int)val_s.toInt();
    apply_one(wheel, value);
    Serial.printf("SET %s = %d\n", wheel.c_str(), value);
    // ให้เห็นทันที 1 บรรทัด
    print_enc_json();
  } else {
    Serial.println("Invalid command");
  }

  line = "";
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial2.begin(ENC_BAUD, SERIAL_8N1, ENC_RX_PIN, ENC_TX_PIN);

  pinMode(PWM_FL, OUTPUT); pinMode(DIR_FL, OUTPUT);
  pinMode(PWM_FR, OUTPUT); pinMode(DIR_FR, OUTPUT);
  pinMode(PWM_RL, OUTPUT); pinMode(DIR_RL, OUTPUT);
  pinMode(PWM_RR, OUTPUT); pinMode(DIR_RR, OUTPUT);

  stop_all();

  Serial.println("=== Motor PWM Test + Continuous Encoder Stream ===");
  Serial.println("Commands:");
  Serial.println("  fl 80 | fl -80 | fr 90 | rl 60 | rr 70");
  Serial.println("  stop");
  Serial.println("  show");
  Serial.println("  stream on/off");
  Serial.println("  rate 100     (ms)");
}

void loop() {
  // 1) update encoder data continuously
  pump_encoder_uart2();

  // 2) print enc data continuously (non-blocking)
  const uint32_t now = millis();
  if (stream_on && (now - last_stream_ms >= stream_period_ms)) {
    last_stream_ms = now;
    print_enc_json();
  }

  // 3) read USB serial commands
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      parse_usb_command_line();
      return;
    } else {
      line += c;
    }
  }
}