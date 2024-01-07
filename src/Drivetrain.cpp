#include "Drivetrain.h"

Drivetrain::Drivetrain(NoU_Motor* frontLeftMotor, NoU_Motor* frontRightMotor, NoU_Motor* backLeftMotor, NoU_Motor* backRightMotor) 
                                    : _frontLeftMotor(frontLeftMotor), _frontRightMotor(frontRightMotor), _backLeftMotor(backLeftMotor), _backRightMotor(backRightMotor)
{ }

void Drivetrain::begin(){
  _frontLeftMotor->set(0);
  _frontRightMotor->set(0);
  _backLeftMotor->set(0);
  _backRightMotor->set(0);
}

void Drivetrain::drive(double power) {
  _frontLeftMotor->set(power);
  _frontRightMotor->set(power);
  _backLeftMotor->set(power);
  _backRightMotor->set(power);
}