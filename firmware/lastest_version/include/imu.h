#ifndef IMU_H
#define IMU_H

struct ImuData {
  float qx, qy, qz, qw;
  float gx, gy, gz;
  float ax, ay, az;
};

void imu_begin();
ImuData get_imu_data();

#endif