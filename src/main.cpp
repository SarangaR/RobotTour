#include <Arduino.h>
#include <Alfredo_NoU2.h>
#include <ESP32Encoder.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

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

PID driveLeftPID = PID(0.1, 0.1, 0, 0);
PID driveRightPID = PID(0.1, 0.1, 0, 0);
PID turnPID = PID(0.1, 0.1, 0, 0);

double i = 0;


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
  Serial.begin(115200);
  // start RSL
  pinMode(LED_BUILTIN, OUTPUT);

  // set direction of motors
  frontLeftMotor.setInverted(false);
  frontRightMotor.setInverted(true);
  backLeftMotor.setInverted(false);
  backRightMotor.setInverted(true);

  // init encoders
  frontLeftEncoder.attachHalfQuad(34, 35);
  frontRightEncoder.attachHalfQuad(36, 39);
  frontLeftEncoder.setCount(0);
  frontRightEncoder.setCount(0);

  double driveLeftKp = 0.1;
  double driveLeftKi = 0;
  double driveLeftKd = 0;

  double driveRightKp = 0.1;
  double driveRightKi = 0;
  double driveRightKd = 0;

  double turnKp = 1;
  double turnKi = 0;
  double turnKd = 0;

  driveLeftPID = PID(0.1, driveLeftKp, driveLeftKi, driveLeftKd);
  driveRightPID = PID(0.1, driveRightKp, driveRightKi, driveRightKd);
  turnPID = PID(0.1, turnKp, turnKi, turnKd);
}

////////////////////////////////////////////////////////////////////// loop() //////////////////////////////////////////////////////////////////////
void loop() {
  if (i == 0) {
    driveInches(12);
    i++;
  }
  // drivetrain.drive(1, 1);
}
////////////////////////////////////////////////////////////////////// Function Code //////////////////////////////////////////////////////////////////////
void driveInches(double inches) {
  double revsperin = 1.0/16.33628;
  double requiredrot = revsperin * inches;
  
  // 40 ticks per rev
  double leftRot = fabs(frontLeftEncoder.getCount() / 40.0); // rotations
  double rightRot = fabs(frontRightEncoder.getCount() / 40.0); // rotations

  double leftTarget = leftRot + requiredrot;
  double rightTarget = rightRot + requiredrot;

  while (!driveLeftPID.isDone(leftTarget, leftRot) && !driveRightPID.isDone(rightTarget, rightRot)) {
    leftRot = fabs(frontLeftEncoder.getCount() / 40.0); // rotations
    rightRot = fabs(frontRightEncoder.getCount() / 40.0); // rotations

    double revsPerSecondLeft = driveLeftPID.calculate(leftTarget, leftRot);
    double revsPerSecondRight = driveRightPID.calculate(rightTarget, rightRot);

    double leftPower = revsPerSecondLeft * 0.3;
    double rightPower = revsPerSecondRight * 0.3;

    // Serial.println("Left Power: " + String(leftPower) + " Right Power: " + String(rightPower));

    drivetrain.drive(leftPower, rightPower);
  }

  drivetrain.drive(0, 0);
}