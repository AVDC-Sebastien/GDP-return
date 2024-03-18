#include <Arduino.h>
#include <ESP32Servo.h>
#include <cmath>
#include <QuickPID.h>

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

// Last control loop time
ulong lastControlMicros = micros();

// Control variables
float roll = 0.f, rollRate = 0.f;
float rollSetpoint = 0.f, rollRateSetpoint = 0.f;

float pitch = 0.f, pitchRate = 0.f;
float pitchSetpoint = 0.f, pitchRateSetpoint = 0.f;

float yaw = 0.f, yawRate = 0.f;
float yawSetpoint = 0.f, yawRateSetpoint = 0.f;

float posZ = 0.f, velZ = 0.f, accZ = 0.f;
float posZSetpoint = 0.f, velZSetpoint = 0.f, accZSetpoint = 0.f;

// Control cmds
float cmdT = 0.f, cmdRoll = 0.f, cmdPitch = 0.f, cmdYaw = 0.f;

// PIDs
QuickPID rollRatePID(&rollRate, &cmdRoll, &rollRateSetpoint, 1.f, 1.f, 1.f, QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas, QuickPID::iAwMode::iAwClamp, QuickPID::Action::direct);
QuickPID rollPID(&roll, &rollRateSetpoint, &rollSetpoint, 1.f, 1.f, 1.f, QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas, QuickPID::iAwMode::iAwClamp, QuickPID::Action::direct);

QuickPID pitchRatePID(&pitchRate, &cmdPitch, &pitchRateSetpoint, 1.f, 1.f, 1.f, QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas, QuickPID::iAwMode::iAwClamp, QuickPID::Action::direct);
QuickPID pitchPID(&pitch, &pitchRateSetpoint, &pitchSetpoint, 1.f, 1.f, 1.f, QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas, QuickPID::iAwMode::iAwClamp, QuickPID::Action::direct);

QuickPID yawRatePID(&yawRate, &cmdYaw, &yawRateSetpoint, 1.f, 1.f, 1.f, QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas, QuickPID::iAwMode::iAwClamp, QuickPID::Action::direct);
QuickPID yawPID(&yaw, &yawRateSetpoint, &yawSetpoint, 1.f, 1.f, 1.f, QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas, QuickPID::iAwMode::iAwClamp, QuickPID::Action::direct);

QuickPID accZPID(&accZ, &cmdT, &accZSetpoint, 1.f, 1.f, 1.f, QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas, QuickPID::iAwMode::iAwClamp, QuickPID::Action::direct);
QuickPID velZPID(&velZ, &accZSetpoint, &velZSetpoint, 1.f, 1.f, 1.f, QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas, QuickPID::iAwMode::iAwClamp, QuickPID::Action::direct);
QuickPID posZPID(&posZ, &velZSetpoint, &posZSetpoint, 1.f, 1.f, 1.f, QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas, QuickPID::iAwMode::iAwClamp, QuickPID::Action::direct);

// Function definitions
int clamp(int, int, int);
float clamp(float, float, float);
void initServoLib();
void setupPID();
void cmdThrust(float, float);
void primeESCs();
void controlLoop();

void setup() {
  // Init Serial
  Serial.begin(115200);

  // Initialize Servo Library
  initServoLib();
  // Prime ESCs
  primeESCs();
  // Setup PIDs
  setupPID();
}

void loop() {

  // Control Loop
  controlLoop();
  
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

void primeESCs() {
  delay(3000);
  Serial.println("Priming ESCs in 5 seconds!");
  delay(5000);
  cmdThrust(1.f, 0.f);
  delay(1000);
  cmdThrust(0.f, 0.f);
}

void controlLoop() {

  // Wait to maintain 50 Hz
  while (micros() < lastControlMicros + 20000);
  lastControlMicros = micros();

  rollRatePID.Compute();
  rollPID.Compute();

  pitchRatePID.Compute();
  pitchPID.Compute();

  yawRatePID.Compute();
  yawPID.Compute();

  accZPID.Compute();
  velZPID.Compute();
  posZPID.Compute();

}

void setupPID() {

  // Setup Limits
  rollRatePID.SetOutputLimits(-1.f, 1.f);
  rollPID.SetOutputLimits(-1.f, 1.f);

  pitchRatePID.SetOutputLimits(-1.f, 1.f);
  pitchPID.SetOutputLimits(-1.f, 1.f);

  yawRatePID.SetOutputLimits(-1.f, 1.f);
  yawPID.SetOutputLimits(-1.f, 1.f);

  accZPID.SetOutputLimits(-1.f, 1.f);
  velZPID.SetOutputLimits(-1.f, 1.f);
  posZPID.SetOutputLimits(-1.f, 1.f);
  
  // Activate PIDs
  rollRatePID.SetMode(QuickPID::Control::automatic);
  rollPID.SetMode(QuickPID::Control::automatic);

  pitchRatePID.SetMode(QuickPID::Control::automatic);
  pitchPID.SetMode(QuickPID::Control::automatic);

  yawRatePID.SetMode(QuickPID::Control::automatic);
  yawPID.SetMode(QuickPID::Control::automatic);

  accZPID.SetMode(QuickPID::Control::automatic);
  velZPID.SetMode(QuickPID::Control::automatic);
  posZPID.SetMode(QuickPID::Control::automatic);

}




