#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

#include "encoder.h"
#include "encoder_config.h"
#include "buzzer.h"

static constexpr uint32_t TX_PERIOD_MS = 20;   // 50 Hz
static constexpr uint32_t ENC_UPDATE_MS = 10;  // 100 Hz

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

static LineReader lr_cmd;

static uint32_t last_upd_ms = 0;
static uint32_t last_tx_ms  = 0;

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

static void handleMainLine(const char *line) {
  if (strncmp(line, "B:", 2) == 0) {
    float duration_sec = atof(line + 2);
    set_buzzer_duration(duration_sec);
    return;
  }

  float duration_sec = 0.0f;
  if (parseFloatKey(line, "buzzer_duration", duration_sec)) {
    set_buzzer_duration(duration_sec);
  }
}

void setup() {
  Serial.begin(115200);  // local monitor only
  Serial2.begin(ENC_UART_BAUD, SERIAL_8N1, ENC_UART_RX_PIN, ENC_UART_TX_PIN);
  Serial2.setRxBufferSize(256);

  encoder_begin();
  buzzer_begin();

  last_upd_ms = millis();
  last_tx_ms  = millis();
}

void loop() {
  static char cmdline[96];
  if (lr_cmd.read(Serial2, cmdline, sizeof(cmdline))) {
    handleMainLine(cmdline);
  }

  handle_buzzer();

  const uint32_t now = millis();

  if (now - last_upd_ms >= ENC_UPDATE_MS) {
    const float dt_s = (now - last_upd_ms) / 1000.0f;
    last_upd_ms = now;
    encoder_update(dt_s);
  }

  if (now - last_tx_ms >= TX_PERIOD_MS) {
    last_tx_ms = now;

    EncoderData e = encoder_get();
    Serial2.printf(
      "{\"enc_vel\":[%.6f,%.6f,%.6f,%.6f],"
      "\"wheel_cnt\":[%lld,%lld,%lld,%lld],"
      "\"t_ms\":%lu}\n",
      e.w[0].vel_mps, e.w[1].vel_mps, e.w[2].vel_mps, e.w[3].vel_mps,
      (long long)e.w[0].count, (long long)e.w[1].count, (long long)e.w[2].count, (long long)e.w[3].count,
      (unsigned long)millis()
    );
  }
}