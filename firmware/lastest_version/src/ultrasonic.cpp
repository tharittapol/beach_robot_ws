// * ref: https://www.instructables.com/Tutorial-How-to-Build-Range-Detector-Using-Arduino/
#include <Arduino.h>
#include "ultrasonic.h"

#define TRIG_LEFT   18
#define ECHO_LEFT   34
#define TRIG_MID    19
#define ECHO_MID    39
#define TRIG_RIGHT  23
#define ECHO_RIGHT  36


void ultrasonic_begin() {
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  
  pinMode(TRIG_MID, OUTPUT);
  pinMode(ECHO_MID, INPUT);
  
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
}


float read_distance_m(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH); 
  float distance_cm = (duration/2) / 29.1;

  if (distance_cm >= 200.0 || distance_cm <= 0.0) {
    return 0.0;
  }
  return distance_cm / 100.0; // * convert to meters 
}


UltrasonicData get_ultrasonic_data() {
  UltrasonicData data;
  
  data.left   = read_distance_m(TRIG_LEFT, ECHO_LEFT);
  data.middle = read_distance_m(TRIG_MID, ECHO_MID);
  data.right  = read_distance_m(TRIG_RIGHT, ECHO_RIGHT);
  
  return data;
}