#ifndef Drivetrain_h
#define Drivetrain_h

#include "Arduino.h"
#include "Motor.h"
#include "PID_v1.h"
#include "ICM_20948.h"

class Drivetrain {
    private:
        Motor left;
        Motor right;

        double input, output, setpoint;
        double imu_kp, imu_ki, imu_kd;

        ICM_20948 imu;

        PID turnPID;

    public:
        Drivetrain(Motor* left, Motor* right, ICM_20948* imu);
        void set(double leftSpeed, double rightSpeed);
        void update();
        void driveDistance(double distance, int units);
        void drive(double speed);
        void turnSpeed(double speed);
        void turn(double angle);
        void stop();
    
};
#endif  