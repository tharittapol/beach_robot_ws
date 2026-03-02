#include <Arduino.h>
#include "encoder.h"
#include "encoder_config.h"

static uint32_t last_enc_ms = 0;
static uint32_t last_pub_ms = 0;

void setup() {
  Serial.begin(115200);
  delay(300);

  encoder_begin();
  encoder_reset();

  last_enc_ms = millis();
  last_pub_ms = millis();

  // Print quick config summary once
  Serial.println("Encoder-only started.");
  Serial.printf("Round wheel R=%.4f m (D=%.2f cm)\n", WCFG[0].radius_m, (WCFG[0].radius_m * 2.0f * 100.0f));
  Serial.printf("Track effective R=%.4f m (D=%.2f cm)\n", WCFG[2].radius_m, (WCFG[2].radius_m * 2.0f * 100.0f));
}

void loop() {
  const uint32_t now = millis();

  // update 10ms (velocity smooth)
  if (now - last_enc_ms >= 10) {
    const float dt_s = (now - last_enc_ms) / 1000.0f;
    last_enc_ms = now;
    encoder_update(dt_s);
  }

  // print 100ms
  if (now - last_pub_ms >= 100) {
    last_pub_ms = now;

    EncoderData enc = get_encoder_data();

    Serial.println("{");
    Serial.printf("  \"enc_counts\":[%lld,%lld,%lld,%lld],\n",
      (long long)enc.w[0].count, (long long)enc.w[1].count,
      (long long)enc.w[2].count, (long long)enc.w[3].count);

    Serial.printf("  \"enc_pos_m\":[%.6f,%.6f,%.6f,%.6f],\n",
      enc.w[0].position_m, enc.w[1].position_m, enc.w[2].position_m, enc.w[3].position_m);

    Serial.printf("  \"enc_vel_mps\":[%.6f,%.6f,%.6f,%.6f],\n",
      enc.w[0].velocity_mps, enc.w[1].velocity_mps, enc.w[2].velocity_mps, enc.w[3].velocity_mps);

    Serial.printf("  \"enc_rpm_output\":[%.3f,%.3f,%.3f,%.3f]\n",
      enc.w[0].rpm_output, enc.w[1].rpm_output, enc.w[2].rpm_output, enc.w[3].rpm_output);
    Serial.println("}");
  }
}