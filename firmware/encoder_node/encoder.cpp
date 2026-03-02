#include "encoder.h"
#include "encoder_config.h"
#include "esp_idf_version.h"

#if ESP_IDF_VERSION_MAJOR >= 5
  #include "driver/pulse_cnt.h"
#else
  #error "This code targets Arduino-ESP32 v3.x (ESP-IDF 5). If you use v2.x tell me."
#endif

struct EncState {
  int64_t total = 0;
  float vel_f = 0.0f;

  pcnt_unit_handle_t unit = nullptr;
  pcnt_channel_handle_t ch_a = nullptr;
  pcnt_channel_handle_t ch_b = nullptr;
};

static EncState S[4];

static inline float counts_per_encoder_rev(const WheelConfig& c) {
  return (float)c.encoder_ppr * (float)c.decode_x;
}

static inline float encoder_rev_per_drive_rev(const WheelConfig& c) {
  if (c.encoderTeeth <= 0.0f) return 1.0f;
  return c.driveTeeth / c.encoderTeeth;
}

static inline float counts_per_output_rev(const WheelConfig& c) {
  // counts/output = counts/enc_rev * enc_rev/drive_rev * drive_rev/output_rev
  return counts_per_encoder_rev(c) * encoder_rev_per_drive_rev(c) * c.driveRevPerOutputRev;
}

static inline float meters_per_output_rev(const WheelConfig& c) {
  // MODEL_ROUND only
  return 2.0f * PI_F * c.radius_m;
}

static inline float meters_per_count(const WheelConfig& c) {
  const float cpor = counts_per_output_rev(c);
  return (cpor > 1e-6f) ? (meters_per_output_rev(c) / cpor) : 0.0f;
}

static bool pcnt_begin(int idx) {
  pcnt_unit_config_t unit_cfg = {};
  unit_cfg.high_limit = 30000;
  unit_cfg.low_limit  = -30000;
  if (pcnt_new_unit(&unit_cfg, &S[idx].unit) != ESP_OK) return false;

  pcnt_glitch_filter_config_t filt = {};
  filt.max_glitch_ns = PCNT_GLITCH_NS;
  if (pcnt_unit_set_glitch_filter(S[idx].unit, &filt) != ESP_OK) return false;

  pcnt_chan_config_t a_cfg = {};
  a_cfg.edge_gpio_num  = WCFG[idx].pinA;
  a_cfg.level_gpio_num = WCFG[idx].pinB;
  if (pcnt_new_channel(S[idx].unit, &a_cfg, &S[idx].ch_a) != ESP_OK) return false;

  pcnt_chan_config_t b_cfg = {};
  b_cfg.edge_gpio_num  = WCFG[idx].pinB;
  b_cfg.level_gpio_num = WCFG[idx].pinA;
  if (pcnt_new_channel(S[idx].unit, &b_cfg, &S[idx].ch_b) != ESP_OK) return false;

  // x4 quadrature actions
  if (pcnt_channel_set_edge_action(S[idx].ch_a,
      PCNT_CHANNEL_EDGE_ACTION_DECREASE,
      PCNT_CHANNEL_EDGE_ACTION_INCREASE) != ESP_OK) return false;

  if (pcnt_channel_set_level_action(S[idx].ch_a,
      PCNT_CHANNEL_LEVEL_ACTION_KEEP,
      PCNT_CHANNEL_LEVEL_ACTION_INVERSE) != ESP_OK) return false;

  if (pcnt_channel_set_edge_action(S[idx].ch_b,
      PCNT_CHANNEL_EDGE_ACTION_INCREASE,
      PCNT_CHANNEL_EDGE_ACTION_DECREASE) != ESP_OK) return false;

  if (pcnt_channel_set_level_action(S[idx].ch_b,
      PCNT_CHANNEL_LEVEL_ACTION_KEEP,
      PCNT_CHANNEL_LEVEL_ACTION_INVERSE) != ESP_OK) return false;

  if (pcnt_unit_enable(S[idx].unit) != ESP_OK) return false;
  if (pcnt_unit_clear_count(S[idx].unit) != ESP_OK) return false;
  if (pcnt_unit_start(S[idx].unit) != ESP_OK) return false;

  return true;
}

static inline int32_t pcnt_read_delta_clear(int idx) {
  int v = 0;
  pcnt_unit_get_count(S[idx].unit, &v);
  pcnt_unit_clear_count(S[idx].unit);
  return (int32_t)v;
}

void encoder_begin() {
  for (int i = 0; i < 4; i++) {
    pinMode(WCFG[i].pinA, INPUT);
    pinMode(WCFG[i].pinB, INPUT);
  }
  for (int i = 0; i < 4; i++) {
    if (!pcnt_begin(i)) {
      Serial.printf("PCNT init failed ENC%d\n", i+1);
      while (true) delay(1000);
    }
  }
  encoder_reset();
}

void encoder_reset() {
  for (int i = 0; i < 4; i++) {
    S[i].total = 0;
    S[i].vel_f = 0;
    pcnt_unit_clear_count(S[i].unit);
  }
}

void encoder_update(float dt_s) {
  if (dt_s <= 0.0f) return;

  for (int i = 0; i < 4; i++) {
    int32_t delta = pcnt_read_delta_clear(i);
    delta *= WCFG[i].sign;

    S[i].total += (int64_t)delta;

    const float v = ((float)delta / dt_s) * meters_per_count(WCFG[i]);
    const float a = WCFG[i].vel_lpf_alpha;
    S[i].vel_f = a * v + (1.0f - a) * S[i].vel_f;
  }
}

EncoderData encoder_get() {
  EncoderData d{};
  for (int i = 0; i < 4; i++) {
    d.w[i].count = S[i].total;
    d.w[i].vel_mps = S[i].vel_f;
  }
  return d;
}