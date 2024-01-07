#include <Arduino.h>
#include <Alfredo_NoU2.h>
#include <ESP32Encoder.h>

#include "IMU.h"
#include "Drivetrain.h"

#include "PID.h"


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

PID driveLeftPID;
PID driveRightPID;
PID turnPID;


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

  double driveLeftKp = 0.1;
  double driveLeftKi = 0;
  double driveleftKd = 0;

  double driveRightKp = 0.1;
  double driveRightKi = 0;
  double driveRightKd = 0;

  double turnKp = 0.1;
  double turnKi = 0;
  double turnKd = 0;

  driveLeftPID = PID(0.1, driveLeftKp, driveLeftKi, driveLeftKd);
  driveRightPID = PID(0.1, driveRightKp, driveRightKi, driveRightKd);
  turnPID = PID(0.1, turnKp, turnKi, turnKd);
}

////////////////////////////////////////////////////////////////////// loop() //////////////////////////////////////////////////////////////////////
void loop() {

}
////////////////////////////////////////////////////////////////////// Function Code //////////////////////////////////////////////////////////////////////
void driveInches(double inches, double power) {
  resetEncoders();
  double revsperin = 1.0/5.93689;
  double requiredrot = revsperin * inches;
  
  // 40 ticks per rev
  double leftRot = fabs(frontLeftEncoder.getCount() / 40.0);
  double rightRot = fabs(frontRightEncoder.getCount() / 40.0);

  rightRot = fabs(frontRightEncoder.getCount() / 40.0); 

  double leftTarget = leftRot + requiredrot;
  double rightTarget = rightRot + requiredrot;

  double ticksPerSecondLeft = driveLeftPID.calculate(leftTarget, leftRot);
  double ticksPerSecondRight = driveRightPID.calculate(rightTarget, rightRot);

  double leftPower = ticksPerSecondLeft / 40.0;
  double rightPower = ticksPerSecondRight / 40.0;

  drivetrain.drive(0);
}