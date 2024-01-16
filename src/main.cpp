#include <Arduino.h>
#include <Alfredo_NoU2.h>
#include <ESP32Encoder.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include "IMU.h"
#include "Drivetrain.h"

#include "PID.h"
#include "PID_RT.h"


/////////////////////////////////// Function Declarations ///////////////////////////////////

// function def
void driveInches(double power);
void turnDegrees(double degrees);

// define motors
NoU_Motor frontLeftMotor(3);
NoU_Motor frontRightMotor(2);
NoU_Motor backLeftMotor(4);
// NoU_Motor backRightMotor(1);
NoU_Motor backRightMotor(5);

// define drivetrain
Drivetrain drivetrain = Drivetrain(&frontLeftMotor, &frontRightMotor, &backLeftMotor, &backRightMotor);

// define encoders
ESP32Encoder frontLeftEncoder; // pins 34, 35
ESP32Encoder frontRightEncoder; // pins 36, 39

PID driveLeftPID = PID(0.1, 0.1, 0, 0);
PID driveRightPID = PID(0.1, 0.1, 0, 0);
PID turnPID = PID(0.1, 0.1, 0, 0);

PID_RT turnPID_RT;

double i = 0;

double glob_yaw_offset = 0;


// define imu object
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

/////////////////////////////////// Logic Declarations ///////////////////////////////////

// enable logic and debounce
void resetEncoders() {
  frontLeftEncoder.setCount(0);
  frontRightEncoder.setCount(0);
}

double getYaw() {
    return bno.getVector(Adafruit_BNO055::VECTOR_EULER).x() - glob_yaw_offset;
}

////////////////////////////////////////////////////////////////////// setup() //////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  // start RSL
  pinMode(LED_BUILTIN, OUTPUT);

  if (!bno.begin()) {
    Serial.println("Could not find BNO055!");
    while (1);
  }

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

  double turnKp =0.1;
  double turnKi = 0;
  double turnKd = 0;

  i=0;

  driveLeftPID = PID(0.1, driveLeftKp, driveLeftKi, driveLeftKd);
  driveRightPID = PID(0.1, driveRightKp, driveRightKi, driveRightKd);
  turnPID = PID(0.1, turnKp, turnKi, turnKd);

  turnPID_RT.setOutputRange(-1, 1);
  turnPID_RT.setInterval(50);
  turnPID_RT.setK(0.1, 0, 0);
  turnPID_RT.start();

  glob_yaw_offset = bno.getVector(Adafruit_BNO055::VECTOR_EULER).x();
}

////////////////////////////////////////////////////////////////////// loop() //////////////////////////////////////////////////////////////////////
void loop() {
    drivetrain.drive(1, 1);
}
////////////////////////////////////////////////////////////////////// Function Code //////////////////////////////////////////////////////////////////////
void driveInches(double inches) {
  double revsperin = 1.0/16.33628;
  double requiredrot = revsperin * inches;
  
  // 40 ticks per rev
  double leftRot = frontLeftEncoder.getCount() / 40.0; // rotations
  double rightRot = frontRightEncoder.getCount() / 40.0; // rotations

  double leftTarget = leftRot + requiredrot;
  double rightTarget = rightRot + requiredrot;

  while (!driveLeftPID.isDone(leftTarget, leftRot) && !driveRightPID.isDone(rightTarget, rightRot)) {
    leftRot = frontLeftEncoder.getCount() / 40.0; // rotations
    rightRot = frontRightEncoder.getCount() / 40.0; // rotations

    double revsPerSecondLeft = driveLeftPID.calculate(leftTarget, leftRot);
    double revsPerSecondRight = driveRightPID.calculate(rightTarget, rightRot);

    double leftPower = revsPerSecondLeft * 0.3;
    double rightPower = revsPerSecondRight * 0.3;

    // Serial.println("Left Power: " + String(leftPower) + " Right Power: " + String(rightPower));

    drivetrain.drive(leftPower, rightPower);
    Serial.println("Left Power: " + String(leftPower) + " Right Power: " + String(rightPower));
  }

  drivetrain.drive(0, 0);
}

void turnDegrees(double degrees) {
    double yaw = getYaw();

    double targetYaw = yaw + degrees;

    turnPID_RT.setPoint(targetYaw);

    while (turnPID_RT.compute(yaw)) {
        yaw = getYaw();

        double power = turnPID_RT.getOutput();

        double roundedPower = round(power * 100) / 100;

        Serial.println("Yaw: " + String(yaw) + " Power: " + String(roundedPower));

        drivetrain.turn(power);
    }

    drivetrain.drive(0, 0);
}
