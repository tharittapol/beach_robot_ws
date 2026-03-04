#include <Arduino.h>
#include "encoder.h"
#include "encoder_config.h"
#include "buzzer.h"

// ส่งข้อมูลไป MAIN ผ่าน UART2 (Serial2)
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

  // --- 3. ส่วนรับคำสั่งเปิด Buzzer จาก main_esp ผ่าน Serial2 ---
  if (Serial2.available() > 0) {
    String cmd = Serial2.readStringUntil('\n');
    cmd.trim(); // ตัด \r \n ช่องว่างทิ้ง
    
    // ตรวจสอบว่าเป็นคำสั่ง Buzzer หรือไม่ (รูปแบบ "B:1.5")
    if (cmd.startsWith("B:")) {
      float duration_sec = cmd.substring(2).toFloat();
      set_buzzer_duration(duration_sec); // โยนไปให้ buzzer.cpp จัดการ
    }
  }

  // --- 4. เช็คเพื่อดับ Buzzer เมื่อครบเวลา ---
  handle_buzzer();


  

  // update @100Hz
  if (now - last_upd_ms >= 10) {
    const float dt_s = (now - last_upd_ms) / 1000.0f;
    last_upd_ms = now;
    encoder_update(dt_s);
  }

  // tx @50Hz
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