/* --- LIBRARIES --- */
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <RunningMedian.h>

const int THROTTLE_POS_PIN = A8;
const int THROTTLE_POS_MIN_PIN = A9;
const int THROTTLE_POS_MAX_PIN = A10;

/// Throttle
const int THROTTLE_POS_MIN = 274;
const int THROTTLE_POS_MAX = 980;
const int THROTTLE_MIN = 0;
const int THROTTLE_MAX = 1024;
const int THROTTLE_MILLIAMP_THRESHOLD = 15000;
const int THROTTLE_GAIN = 10 ;
const int THROTTLE_STEP = 64;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *throttle_motor = AFMS.getMotor(3);

void setup() {
  pinMode(THROTTLE_POS_PIN, INPUT);
  pinMode(THROTTLE_POS_MIN_PIN, OUTPUT);
  digitalWrite(THROTTLE_POS_MIN_PIN, LOW);
  pinMode(THROTTLE_POS_MAX_PIN, OUTPUT);
  digitalWrite(THROTTLE_POS_MAX_PIN, HIGH);
}

void loop() {
  int pos = 500; // this is the number that you will use to the set the position
  set_throttle(pos);
}

int set_throttle(int setpoint) {
  int in = map(analogRead(THROTTLE_POS_PIN), THROTTLE_POS_MIN, THROTTLE_POS_MAX, 1024, 0); // get the position feedback from the linear actuator
  int error = in - setpoint; // the error between what you have and what you want
  int out = -1 * THROTTLE_GAIN * map(error, -1024, 1024, -255, 255); // the output to the motor controller

  // Engage throttle actuator
  Serial.println(out);
  if (out > 0) {
    Serial.println("up throttle");
    throttle_motor->setSpeed(out);
    throttle_motor->run(FORWARD);
    delay(20);
  }
  else if (out < 0) {
    Serial.println("down throttle");
    throttle_motor->setSpeed(abs(out));
    throttle_motor->run(BACKWARD);
    delay(20);
  }
}
