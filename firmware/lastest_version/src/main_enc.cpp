#include <Arduino.h>
#include "encoder.h"
#include "encoder_config.h"
#include "buzzer.h"

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
static LineReader lr_cmd;

// schedule
static uint32_t last_upd_ms = 0;
static uint32_t last_tx_ms  = 0;

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial2.begin(ENC_UART_BAUD, SERIAL_8N1, ENC_UART_RX_PIN, ENC_UART_TX_PIN);

  encoder_begin();
  last_upd_ms = millis();
  last_tx_ms  = millis();

  buzzer_begin();
}

void loop() {
  const uint32_t now = millis();

  // 1) RX from MAIN (buzzer)
  static char cmdline[64];
  if (lr_cmd.read(Serial2, cmdline, sizeof(cmdline))) {
    if (strncmp(cmdline, "B:", 2) == 0) {
      float duration_sec = atof(cmdline + 2);
      set_buzzer_duration(duration_sec);
    }
  }
  handle_buzzer();

  // 2) update encoder @100Hz
  if (now - last_upd_ms >= 10) {
    const float dt_s = (now - last_upd_ms) / 1000.0f;
    last_upd_ms = now;
    encoder_update(dt_s);
  }

  // 3) tx to MAIN @50Hz (สบาย ๆ ที่ 115200)
  if (now - last_tx_ms >= 20) {
    last_tx_ms = now;

    EncoderData e = encoder_get();
    Serial2.printf(
      "{\"enc_vel\":[%.6f,%.6f,%.6f,%.6f],\"wheel_cnt\":[%lld,%lld,%lld,%lld],\"t_ms\":%lu}\n",
      e.w[0].vel_mps, e.w[1].vel_mps, e.w[2].vel_mps, e.w[3].vel_mps,
      (long long)e.w[0].count, (long long)e.w[1].count, (long long)e.w[2].count, (long long)e.w[3].count,
      (unsigned long)millis()
    );
  }
}