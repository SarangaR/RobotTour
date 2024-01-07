#include <Arduino.h>
#include <Alfredo_NoU2.h>
#include <ESP32Encoder.h>

#include "IMU.h"
#include "Drivetrain.h"
#include "Arm.h"

/////////////////////////////////// Function Declarations ///////////////////////////////////

// function def
void driveInches(double power);

// define motors
NoU_Motor frontLeftMotor(3);
NoU_Motor frontRightMotor(2);
NoU_Motor backLeftMotor(4);
NoU_Motor backRightMotor(1);

// define drivetrain
Drivetrain drivetrain = Drivetrain(&frontLeftMotor, &frontRightMotor, &backLeftMotor, &backRightMotor);

// define encoders
ESP32Encoder frontLeftEncoder; // pins 34, 35
ESP32Encoder frontRightEncoder; // pins 36, 39


// define imu object
// IMU imu;

/////////////////////////////////// Logic Declarations ///////////////////////////////////

// enable logic and debounce
void resetEncoders() {
  frontLeftEncoder.setCount(0);
  frontRightEncoder.setCount(0);
}

////////////////////////////////////////////////////////////////////// setup() //////////////////////////////////////////////////////////////////////
void setup() {
  // start arm

  // start RSL
  pinMode(LED_BUILTIN, OUTPUT);

  // set direction of motors
  frontLeftMotor.setInverted(false);
  frontRightMotor.setInverted(false);
  backLeftMotor.setInverted(true);
  backRightMotor.setInverted(true);

  // init encoders
  frontLeftEncoder.attachHalfQuad(34, 35);
  frontRightEncoder.attachHalfQuad(36, 39);
  frontLeftEncoder.setCount(0);
  frontRightEncoder.setCount(0);
}
////////////////////////////////////////////////////////////////////// loop() //////////////////////////////////////////////////////////////////////
void loop() {

}
////////////////////////////////////////////////////////////////////// Function Code //////////////////////////////////////////////////////////////////////
void driveInches(double inches, double linearX, double linearY, double angularZ) {
  resetEncoders();
  double revsperin = 1.0/5.93689;
  double requiredrot = revsperin * inches;
  
  // 40 ticks per rev
  double leftRot = fabs(frontLeftEncoder.getCount() / 40.0);
  double rightRot = fabs(frontRightEncoder.getCount() / 40.0);

  rightRot = fabs(frontRightEncoder.getCount() / 40.0); 

  double leftTarget = leftRot + requiredrot;
  double rightTarget = rightRot + requiredrot;

  serialBT.println("leftRot: " + String(leftRot) + "rightRot: " + String(rightRot));
  serialBT.println("leftTarget: " + String(leftTarget) + "rightTarget: " + String(rightTarget));

  while (rightRot < rightTarget || leftRot < leftTarget) {
    leftRot = fabs(frontLeftEncoder.getCount() / 40.0);
    rightRot = fabs(frontRightEncoder.getCount() / 40.0);
    AlfredoConnect.update();
    if (AlfredoConnect.keyHeld(Key::Space)) {
      return;
    } 
    serialBT.println("leftRot: " + String(leftRot) + "rightRot: " + String(rightRot));
    serialBT.println("leftTarget: " + String(leftTarget) + "rightTarget: " + String(rightTarget));
    drivetrain.set(linearX, linearY, angularZ);
  }
  drivetrain.set(0, 0, 0);
}