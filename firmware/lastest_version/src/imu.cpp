// * ref: https://www.massmore.shop/bno055-bno085-bno086-%e0%b8%8a%e0%b8%b4%e0%b8%9e%e0%b9%81%e0%b8%97%e0%b9%89-%e0%b8%a3%e0%b8%b8%e0%b9%88%e0%b8%99-halley-9-dof-imu-sensor-bosch-ceva-by-massmore/
#include "imu.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


void imu_begin() {  
  Wire.begin(21, 22);

  while (!bno.begin()) {
    Serial.println("BNO055 not detected!");
    delay(200);
  }

  delay(1000);
  bno.setExtCrystalUse(true);
}


ImuData get_imu_data() {
  ImuData data;
  
  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> lin_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  data.qx = quat.x(); data.qy = quat.y(); data.qz = quat.z(); data.qw = quat.w();
  data.gx = gyro.x(); data.gy = gyro.y(); data.gz = gyro.z();
  data.ax = lin_acc.x(); data.ay = lin_acc.y(); data.az = lin_acc.z();

  return data;
}