#include <Arduino.h>

#include "initial.h"
#include "imu.h"
#include "ultrasonic.h"
#include "motor.h"


void system_begin() {
  Serial.begin(115200);
  
  motor_begin();
  imu_begin();
  ultrasonic_begin();
  
}