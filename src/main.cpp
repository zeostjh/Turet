#include <Arduino.h>
#include <AccelStepper.h>
#include "sbus.h"

// --- Stepper X (A4988 #1) ---
#define STEP_X 4   // D3
#define DIR_X  3   // D2

// --- Stepper Y (A4988 #2) ---
#define STEP_Y 7   // D7
#define DIR_Y  6   // D6

AccelStepper stepperX(AccelStepper::DRIVER, STEP_X, DIR_X);
AccelStepper stepperY(AccelStepper::DRIVER, STEP_Y, DIR_Y);

// --- SBUS Setup ---
#define SBUS_RX_PIN 20
bfs::SbusRx sbus_rx(&Serial1, SBUS_RX_PIN, -1, true);
bfs::SbusData data;

// --- Settings ---
const int deadband = 50;      // SBUS deadband
const int maxPosition = 1000; // max offset
const int accel = 800;        // acceleration (steps/s^2)
const int maxSpeed = 1000;    // maximum speed

// --- Telemetry Cache ---
int lastCh1 = -1;
int lastCh2 = -1;
long lastPosX = -9999;
long lastPosY = -9999;

void setup() {
  Serial.begin(115200);

  // Setup SBUS
  sbus_rx.Begin();

  stepperX.setMaxSpeed(maxSpeed);
  stepperX.setAcceleration(accel);

  stepperY.setMaxSpeed(maxSpeed);
  stepperY.setAcceleration(accel);

  Serial.println("SBUS Turret Control Ready");
}

void loop() {
  // Read SBUS data
  if (sbus_rx.Read()) {
    data = sbus_rx.data();
    
    // Check if receiver is connected and signal is good
    if (!data.lost_frame && !data.failsafe) {
      
      // Get joystick channels (typically Ch1=aileron/roll, Ch2=elevator/pitch)
      int ch1Val = data.ch[0];  // Right stick left/right (X-axis)
      int ch2Val = data.ch[1];  // Right stick forward/back (Y-axis)
      
      int center = 992;  // SBUS center value (range: 172-1811)

      // --- X Speed (Left/Right) - Servo-like behavior ---
      float xSpeed = 0;
      if (abs(ch1Val - center) > deadband) {
        // Map stick position to speed (-maxSpeed to +maxSpeed)
        xSpeed = map(ch1Val, 172, 1811, -maxSpeed, maxSpeed);
        stepperX.setSpeed(xSpeed);
        stepperX.runSpeed();
      } else {
        // Stop smoothly when stick is centered
        stepperX.setSpeed(0);
        stepperX.runSpeed();
      }

      // --- Y Speed (Forward/Back) - Servo-like behavior ---
      float ySpeed = 0;
      if (abs(ch2Val - center) > deadband) {
        // Map stick position to speed (-maxSpeed to +maxSpeed)
        ySpeed = map(ch2Val, 172, 1811, -maxSpeed, maxSpeed);
        stepperY.setSpeed(ySpeed);
        stepperY.runSpeed();
      } else {
        // Stop smoothly when stick is centered
        stepperY.setSpeed(0);
        stepperY.runSpeed();
      }

      // --- Telemetry (only if changed) ---
      long posX = stepperX.currentPosition();
      long posY = stepperY.currentPosition();

      if (ch1Val != lastCh1 || ch2Val != lastCh2 ||
          posX != lastPosX || posY != lastPosY) {

        Serial.print("ch1:"); Serial.print(ch1Val);
        Serial.print("\tch2:"); Serial.print(ch2Val);
        Serial.print("\tspeedX:"); Serial.print(xSpeed);
        Serial.print("\tspeedY:"); Serial.print(ySpeed);
        Serial.print("\tposX:"); Serial.print(posX);
        Serial.print("\tposY:"); Serial.println(posY);

        lastCh1 = ch1Val;
        lastCh2 = ch2Val;
        lastPosX = posX;
        lastPosY = posY;
      }
    } else {
      // Signal lost - stop motors smoothly
      stepperX.setSpeed(0);
      stepperY.setSpeed(0);
      stepperX.runSpeed();
      stepperY.runSpeed();
      
      if (data.lost_frame || data.failsafe) {
        Serial.println("SBUS: Signal lost or failsafe active");
      }
    }
  }
}
