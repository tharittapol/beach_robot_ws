// * ref: https://www.massmore.shop/bno055-bno085-bno086-%e0%b8%8a%e0%b8%b4%e0%b8%9e%e0%b9%81%e0%b8%97%e0%b9%89-%e0%b8%a3%e0%b8%b8%e0%b9%88%e0%b8%99-halley-9-dof-imu-sensor-bosch-ceva-by-massmore/

#include "imu.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

static Adafruit_BNO055 bno(55, 0x28);

static bool s_ready = false;
static uint32_t s_last_retry_ms = 0;
static ImuData s_last = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};

static void try_init() {
  if (bno.begin()) {
    delay(20);
    bno.setExtCrystalUse(true);
    s_ready = true;
    s_last.valid = true;
    s_last.qx = 0.0f;
    s_last.qy = 0.0f;
    s_last.qz = 0.0f;
    s_last.qw = 1.0f;
  } else {
    s_ready = false;
    s_last.valid = false;
  }
}

void imu_begin() {
  Wire.begin(21, 22);
  try_init();
  s_last_retry_ms = millis();
}

void imu_poll() {
  if (!s_ready) {
    const uint32_t now = millis();
    if (now - s_last_retry_ms >= 1000) {
      s_last_retry_ms = now;
      try_init();
    }
    return;
  }

  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> lin_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  s_last.qx = quat.x();
  s_last.qy = quat.y();
  s_last.qz = quat.z();
  s_last.qw = quat.w();

  s_last.gx = gyro.x();
  s_last.gy = gyro.y();
  s_last.gz = gyro.z();

  s_last.ax = lin_acc.x();
  s_last.ay = lin_acc.y();
  s_last.az = lin_acc.z();
  s_last.valid = true;
}

bool imu_is_ready() {
  return s_ready;
}

ImuData get_imu_data() {
  return s_last;
}