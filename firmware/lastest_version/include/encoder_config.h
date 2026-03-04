#pragma once
#include <Arduino.h>

// ========== เลือกโมเดลการแปลง ==========
enum WheelModel : uint8_t {
  MODEL_ROUND = 0,  // linear per rev = 2*pi*R
};

struct WheelConfig {
  // ---- pins (ตาม schematic) ----
  int pinA;
  int pinB;

  // ---- encoder spec ----
  int32_t encoder_ppr;   // RT3806 = 360
  int32_t decode_x;      // 4 recommended (x4)
  int8_t  sign;          // +1 / -1 ถ้าทิศกลับ

  // ---- encoder drive ratio (อัตราทดจาก "drive shaft" ที่ขับ encoder -> encoder) ----
  // encoder_rev_per_drive_rev = drive_teeth / encoder_teeth
  // ถ้า 1:1 ใส่ 1 และ 1
  float driveTeeth;
  float encoderTeeth;

  // ---- สำคัญที่สุด: driveRevPerOutputRev ----
  // = จำนวนรอบของ "drive shaft ที่ขับ encoder" ต่อ 1 รอบของ OUTPUT (ล้อ/สโปรเก็ตขับแทรค)
  //
  // ตัวอย่าง track:
  //   drive shaft = สเตอร์มอเตอร์ 9T (ที่ขับ encoder)
  //   output      = สเตอร์ล้อ 15T
  //   driveRevPerOutputRev = 15/9
  //
  float driveRevPerOutputRev;

  // ---- model ----
  WheelModel model;

  // ---- round wheel effective radius (m) ----
  float radius_m;

  // ---- preprocessing ----
  float vel_lpf_alpha;   // 0..1 (สูง=ตอบสนองไว)
};

// PCNT glitch reject (ns) เพิ่มถ้ามี noise count (เพิ่มได้ถึง 10000)
static constexpr uint32_t PCNT_GLITCH_NS = 5000;

// ===================== Helper =====================
static constexpr float PI_F = 3.14159265358979323846f;

// ===== UART2 (ไป ESP32 main) =====
static constexpr uint32_t ENC_UART_BAUD = 115200;
// UART2 default pins
static constexpr int ENC_UART_RX_PIN = 16;
static constexpr int ENC_UART_TX_PIN = 17;

// Track sprocket -> treat as round: Reff = (pitch * teeth) / (2*pi)
static inline float sprocket_pitch_radius(float pitch_m, int teeth) {
  return (pitch_m * (float)teeth) / (2.0f * PI_F);
}

// ===================== Encoder =====================

// ----- Round wheel -----
static constexpr float ROUND_WHEEL_DIAM_M = 0.3202f; // 33.02 cm
static constexpr float ROUND_WHEEL_R_M    = ROUND_WHEEL_DIAM_M * 0.5f;

// Motor MY1016Z3 gearbox ratio (ใช้เมื่อ encoder อยู่ "ก่อนเกียร์")
static constexpr float GEAR_RATIO = 9.78f; // 9.78:1

// ----- Track wheel -----
static constexpr float TRACK_PITCH_M = 0.035f; // 3.5 cm
static constexpr int   SPROCKET_OUT_TEETH = 15; // wheel sprocket teeth (output)
static constexpr int   SPROCKET_DRIVE_TEETH = 9; // motor sprocket teeth (drive)
static constexpr int   SPROCKET_ENCODER_TEETH = 9; // encoder sprocket teeth

// track effective radius (treat as round drive wheel)
// static constexpr float TRACK_R_EFF_M = (TRACK_PITCH_M * (float)SPROCKET_OUT_TEETH) / (2.0f * PI_F);
static constexpr float TRACK_R_EFF_M = 0.115f; // 11.5 cm (real measure)

// ================== CONFIG 4 WHEELS ==================
// Pins: ENC1(32,33), ENC2(25,26), ENC3(27,14), ENC4(12,13)
// NOTE: GPIO12 เป็น strap pin
static WheelConfig WCFG[4] = {
  // -------- Wheel 1 (FL) : TRACK (treat as ROUND drive sprocket) --------
  {
    .pinA=32, .pinB=33,
    .encoder_ppr=360, .decode_x=4, .sign=+1,
    .driveTeeth=(float)SPROCKET_DRIVE_TEETH, .encoderTeeth=(float)SPROCKET_ENCODER_TEETH, // 9:9 = 1:1
    .driveRevPerOutputRev=3.1180556f,      // counts_per_output_rev / counts_per_encoder_rev 4492/1440
    .model=MODEL_ROUND,
    .radius_m=TRACK_R_EFF_M,
    .vel_lpf_alpha=0.25f
  },

  // -------- Wheel 2 (FR) : TRACK (treat as ROUND drive sprocket) --------
  {
    .pinA=25, .pinB=26,
    .encoder_ppr=360, .decode_x=4, .sign=-1,
    .driveTeeth=(float)SPROCKET_DRIVE_TEETH, .encoderTeeth=(float)SPROCKET_ENCODER_TEETH,
    .driveRevPerOutputRev=2.9625f, // 4266/1440
    .model=MODEL_ROUND,
    .radius_m=TRACK_R_EFF_M,
    .vel_lpf_alpha=0.25f
  },

  // -------- Wheel 3 (RL) : ROUND --------
  {
    .pinA=27, .pinB=14,
    .encoder_ppr=360, .decode_x=4, .sign=+1,
    .driveTeeth=1, .encoderTeeth=1,
    .driveRevPerOutputRev=1.0f,
    .model=MODEL_ROUND,
    .radius_m=ROUND_WHEEL_R_M,
    .vel_lpf_alpha=0.25f
  },

  // -------- Wheel 4 (RR) : ROUND --------
  {
    .pinA=36, .pinB=35,
    .encoder_ppr=360, .decode_x=4, .sign=-1,
    .driveTeeth=1, .encoderTeeth=1,
    .driveRevPerOutputRev=1.0f,
    .model=MODEL_ROUND,
    .radius_m=ROUND_WHEEL_R_M,
    .vel_lpf_alpha=0.25f
  },
};