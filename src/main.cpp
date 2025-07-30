#include <Arduino.h>
#include <AccelStepper.h>

// --- Stepper X (A4988 #1) ---
#define STEP_X 4   // D3
#define DIR_X  3   // D2

// --- Stepper Y (A4988 #2) ---
#define STEP_Y 7   // D7
#define DIR_Y  6   // D6

AccelStepper stepperX(AccelStepper::DRIVER, STEP_X, DIR_X);
AccelStepper stepperY(AccelStepper::DRIVER, STEP_Y, DIR_Y);


#define JOY_X A0
#define JOY_Y A1


const int deadband = 50;
const int maxPosition = 1000;
const int accel = 800;
const int maxSpeed = 1000;

int lastJoyX = -1;
int lastJoyY = -1;
long lastTargetX = -9999;
long lastTargetY = -9999;
long lastPosX = -9999;
long lastPosY = -9999;

void setup() {
  Serial.begin(115200);

  stepperX.setMaxSpeed(maxSpeed);
  stepperX.setAcceleration(accel);

  stepperY.setMaxSpeed(maxSpeed);
  stepperY.setAcceleration(accel);

  analogReadResolution(12); // ESP32-C3 ADC = 0-4095
}

void loop() {
  int xVal = analogRead(JOY_X);
  int yVal = analogRead(JOY_Y);

  int center = 2048;

  // --- X Target ---
  long xTarget = 0;
  if (abs(xVal - center) > deadband) {
    xTarget = map(xVal, 0, 4095, maxPosition, -maxPosition);
  }

  // --- Y Target ---
  long yTarget = 0;
  if (abs(yVal - center) > deadband) {
    yTarget = map(yVal, 0, 4095, maxPosition, -maxPosition);
  }

  // Update steppers
  stepperX.moveTo(xTarget);
  stepperY.moveTo(yTarget);

  stepperX.run();
  stepperY.run();

  // --- Telemetry (only if changed) ---
  long posX = stepperX.currentPosition();
  long posY = stepperY.currentPosition();

  if (xVal != lastJoyX || yVal != lastJoyY ||
      xTarget != lastTargetX || yTarget != lastTargetY ||
      posX != lastPosX || posY != lastPosY) {

    Serial.print("joyX:"); Serial.print(xVal);
    Serial.print("\tjoyY:"); Serial.print(yVal);
    Serial.print("\ttargetX:"); Serial.print(xTarget);
    Serial.print("\ttargetY:"); Serial.print(yTarget);
    Serial.print("\tposX:"); Serial.print(posX);
    Serial.print("\tposY:"); Serial.println(posY);

    lastJoyX = xVal;
    lastJoyY = yVal;
    lastTargetX = xTarget;
    lastTargetY = yTarget;
    lastPosX = posX;
    lastPosY = posY;
  }
}
