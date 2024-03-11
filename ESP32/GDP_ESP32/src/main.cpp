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
void primeESCs();
void normalEDFTest();
void impulseEDFTest();
void sinEDFTest();

void setup() {
  // Init Serial
  Serial.begin(115200);

  // Initialize Servo Library
  initServoLib();
  // Prime ESCs
  primeESCs();

  // TESTS
  normalEDFTest();
  //impulseEDFTest();
  //sinEDFTest();
}

void loop() {

  
  delay(20000);
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

void normalEDFTest() {

  // TEST CONFIG //
  float totalTime = 10.f; // How long should it last in seconds
  float signalUpdateRate = 50.f; // Signal Update Rate in Hz
  /////////////////

  Serial.println("Beginning the normal test in 10 seconds!");
  Serial.print("It will last "); Serial.print(totalTime); Serial.println(" seconds.");
  Serial.print("Singal Update Rate: "); Serial.print(signalUpdateRate); Serial.println(" Hz.");
  delay(10000);

  float increment = 1.f/(signalUpdateRate * totalTime);
  
  ulong stepTimeMicros = (ulong)roundf((1.f/signalUpdateRate)*1000000.f);
  
  ulong nextUpdateMicros = micros();

  ulong startTime = millis();

  for (float i = 0.f; i <= 1.f; i += increment)
  {
    while (micros() < nextUpdateMicros);
    nextUpdateMicros = micros() + stepTimeMicros;

    EDFN.writeMicroseconds(1000 + (int)roundf(i * 1000));
    Serial.println(i, 4);
  }

  ulong timeLapsed = millis() - startTime;
  Serial.print("End of test! It took: ");
  Serial.print((timeLapsed / 1000.f));
  Serial.println(" seconds! Shutting down the EDF...");

  EDFN.writeMicroseconds(1000);
  
}

void impulseEDFTest() {

  // TEST CONFIG //
  float totalTime = 10.f;         // How long should it last in seconds
  float signalUpdateRate = 50.f;  // Signal Update Rate in Hz
  float freqMultiplier = 0.8f;    // How much should signal swap time between high and low decrease
  /////////////////

  Serial.println("Beginning the IMPULSE test in 10 seconds!");
  Serial.print("It will last "); Serial.print(totalTime); Serial.println(" seconds.");
  Serial.print("Singal Update Rate: "); Serial.print(signalUpdateRate); Serial.println(" Hz.");
  delay(10000);

  float increment = 1.f/(signalUpdateRate * totalTime);

  bool isHigh = false;
  
  ulong stepTimeMicros = (ulong)roundf((1.f/signalUpdateRate)*1000000.f);
  
  ulong nextUpdateMicros = micros();

  float swapTime = 1.f;
  ulong nextSwapMicros = micros() + (ulong)roundf(swapTime * 1000.f);

  ulong startTime = millis();

  for (float i = 0.f; i <= (totalTime + (1/signalUpdateRate)); i += (1/signalUpdateRate))
  {
    while (micros() < nextUpdateMicros);
    nextUpdateMicros = micros() + stepTimeMicros;

    // Swap
    if (micros() > nextSwapMicros) {
      nextSwapMicros = micros() + (ulong)roundf(swapTime * 1000000.f);
      isHigh = !isHigh;

      if (!isHigh) swapTime *= freqMultiplier;
    }
      
    
    if (isHigh) {
      EDFN.writeMicroseconds(2000);
      Serial.println(1.f);
    }
    else {
      EDFN.writeMicroseconds(1000);
      Serial.println(0.f);
    }
  }

  ulong timeLapsed = millis() - startTime;
  Serial.print("End of test! It took: ");
  Serial.print((timeLapsed / 1000.f));
  Serial.println(" seconds! Shutting down the EDF...");

  EDFN.writeMicroseconds(1000);
  
}

void sinEDFTest() {

  // TEST CONFIG //
  float totalTime = 10.f;         // How long should it last in seconds
  float signalUpdateRate = 50.f;  // Signal Update Rate in Hz
  float targetFreq = 3.f * (2.f * PI);        // Final frequency in rad/s
  /////////////////

  Serial.println("Beginning the SINE test in 10 seconds!");
  Serial.print("It will last "); Serial.print(totalTime); Serial.println(" seconds.");
  Serial.print("Singal Update Rate: "); Serial.print(signalUpdateRate); Serial.println(" Hz.");
  delay(10000);

  float increment = (1.f/(signalUpdateRate * totalTime)) * (targetFreq - PI);
  
  ulong stepTimeMicros = (ulong)roundf((1.f/signalUpdateRate)*1000000.f);
  ulong nextUpdateMicros = micros();

  ulong startTime = millis();
  
  float timePassed = 0.f;
  for (float i = PI; i <= targetFreq; i += increment)
  {
    while (micros() < nextUpdateMicros);
    nextUpdateMicros = micros() + stepTimeMicros;

    timePassed = (millis() - startTime) / 1000.f;
    
    float thrust = 0.5f + 0.5f * sinf(timePassed * i);
    EDFN.writeMicroseconds(1000 + (int)roundf(thrust * 1000.f));
    Serial.println(thrust);

    if (timePassed >= totalTime) break;
  }

  ulong timeLapsed = millis() - startTime;
  Serial.print("End of test! It took: ");
  Serial.print((timeLapsed / 1000.f));
  Serial.println(" seconds! Shutting down the EDF...");

  EDFN.writeMicroseconds(1000);
}


