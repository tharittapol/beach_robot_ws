#ifndef ULTRASONIC_H
#define ULTRASONIC_H

struct UltrasonicData {
  float left;   // n1
  float middle; // n2
  float right;  // n3
};

void ultrasonic_begin();

UltrasonicData get_ultrasonic_data();

#endif