#pragma once
#include <Arduino.h>

struct EncoderWheel {
  int64_t count;
  float vel_mps;
};

struct EncoderData {
  EncoderWheel w[4];
};

void encoder_begin();
void encoder_reset();
void encoder_update(float dt_s);
EncoderData encoder_get();