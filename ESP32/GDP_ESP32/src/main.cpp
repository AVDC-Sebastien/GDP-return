#include <Arduino.h>
#include <ESP32Servo.h>
#include <cmath>

// Config - Servo Pins
#define PITCH_SERVO_PIN 6
#define ROLL_SERVO_PIN 7

// Define EDF / Servo variables
Servo PitchServo;
Servo RollServo;

// Function definitions
int clamp(int, int, int);
float clamp(float, float, float);
void initServoLib();

void setup() {
  // Init Serial
  Serial.begin(115200);

  // Initialize Servo Library
  initServoLib();
}

void loop() {

  // U fat

}

// Clamp value
int clamp(int value, int minimum, int maximum) {
  return max(minimum, min(maximum, value));
}
float clamp(float value, float minimum, float maximum) {
  return max(minimum, min(maximum, value));
}

// Initialize Servo Library
void initServoLib() {
  PitchServo.attach(PITCH_SERVO_PIN);
  RollServo.attach(ROLL_SERVO_PIN);
}



