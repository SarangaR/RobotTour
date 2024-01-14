#include "Arduino.h"
#include "Drivetrain.h"
#include "Motor.h"
#include "DistanceUtiil.h"
#include "ICM_20948.h"
#include "RobotConfig.h"

Drivetrain::Drivetrain(Motor* left, Motor* right, ICM_20948* imu):
    left(*left),
    right(*right),
    imu(*imu),
    input(0),
    output(0),
    setpoint(0),
    imu_kp(IMU_KP),
    imu_ki(IMU_KI),
    imu_kd(IMU_KD),
    turnPID(PID(&this->input, &this->output, &this->setpoint, this->imu_kp, this->imu_ki, this->imu_kd, DIRECT))
{
    this->turnPID.SetOutputLimits(-1, 1);
}

void Drivetrain::set(double leftSpeed, double rightSpeed) {
    left.set(leftSpeed);
    right.set(rightSpeed);
}

void Drivetrain::drive(double speed) {
    left.set(speed);
    right.set(speed);
}

void Drivetrain::driveDistance(double distance, int units) {
    double distance_ticks = DistanceUtil::distanceToEncoderTicks(distance, units);
    double left_target = left.getPosition() + distance_ticks;
    double right_target = right.getPosition() + distance_ticks;
    left.setPosition(left_target);
    right.setPosition(right_target);
}

void Drivetrain::turnSpeed(double speed) {
    left.set(speed);
    right.set(-speed);
}

void Drivetrain::turn(double angle) {
    this->setpoint = this->imu.gyrZ() + angle;
    while (abs(this->input - this->setpoint) > 0.5) {
        this->setpoint = this->imu.gyrZ() + angle;
        while (!this->imu.dataReady()) {};
        this->input = this->imu.gyrZ();
        this->turnPID.Compute();
        this->turnSpeed(this->output);
    }
}

void Drivetrain::stop() {
    left.stop();
    right.stop();
}

/*
    * Updates the PID controllers for each motor
*/
void Drivetrain::update() {
    left.update();
    right.update();
}