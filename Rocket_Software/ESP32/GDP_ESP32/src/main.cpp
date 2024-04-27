#include <Arduino.h>
#include <ESP32Servo.h>
#include <cmath>

// Config - EDF Pins
#define EDF_N_PIN 1
#define EDF_S_PIN 2
#define EDF_W_PIN 3
#define EDF_E_PIN 5

// Config - Servo Pins
#define PITCH_SERVO_PIN 6
#define ROLL_SERVO_PIN 7

// Define EDF / Servo variables
Servo EDFN;
Servo EDFS;
Servo EDFW;
Servo EDFE;
Servo PitchServo;
Servo RollServo;

float test = -.5f;
ulong curtime = 0;

// Function definitions
int clamp(int, int, int);
float clamp(float, float, float);
void initServoLib();
void cmdThrust(float, float);

void setup() {
  // Init Serial
  Serial.begin(115200);

  // Initialize Servo Library
  initServoLib();
}

void loop() {

  Serial.println(" Hello world!");

  Serial.println(test);
  curtime = micros();
  
  cmdThrust(test, 0.f);
  test += 0.1f;

  ulong diff = micros() - curtime;
  Serial.println(diff);
  
  delay(1000);
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
  EDFN.attach(EDF_N_PIN);
  EDFS.attach(EDF_S_PIN);
  EDFW.attach(EDF_W_PIN);
  EDFE.attach(EDF_E_PIN);
  PitchServo.attach(PITCH_SERVO_PIN);
  RollServo.attach(ROLL_SERVO_PIN);
}


void cmdThrust(float thrust, float yawCtrl) {
  // Clamp input values
  thrust = clamp(thrust, 0.f, 1.f);
  yawCtrl = clamp(yawCtrl, -1.f, 1.f);

  int convertedThrust = (int)roundf(thrust*1000.f);
  int convertedYaw = (int)roundf(yawCtrl*200.f);

  EDFN.writeMicroseconds(1000 + clamp(convertedThrust + convertedYaw, 0, 1000));
  EDFS.writeMicroseconds(1000 + clamp(convertedThrust + convertedYaw, 0, 1000));
  EDFW.writeMicroseconds(1000 + clamp(convertedThrust - convertedYaw, 0, 1000));
  EDFE.writeMicroseconds(1000 + clamp(convertedThrust - convertedYaw, 0, 1000));
}