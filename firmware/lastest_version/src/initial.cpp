#include "initial.h"

#include "imu.h"
#include "ultrasonic.h"
#include "motor.h"

void system_begin() {
  motor_begin();
  imu_begin();
  ultrasonic_begin();
}