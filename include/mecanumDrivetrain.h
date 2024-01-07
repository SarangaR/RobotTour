#ifndef Mecanum_h
#define Mecanum_h

#include <Arduino.h>
#include <Alfredo_NoU2.h>

class Drivetrain
{
    public:
        Drivetrain(NoU_Motor* frontLeftMotor, NoU_Motor* frontRightMotor, NoU_Motor* backLeftMotor, NoU_Motor* backRightMotor);
        void begin();
        
        void drive(double power);

    private:
        NoU_Motor* _frontLeftMotor;
        NoU_Motor* _frontRightMotor;
        NoU_Motor* _backLeftMotor;
        NoU_Motor* _backRightMotor;
        
        float frontLeftCurrentPower;
        float backLeftCurrentPower;
        float frontRightCurrentPower;
        float backRightCurrentPower;

        float frontLeftDesiredPower;
        float backLeftDesiredPower;
        float frontRightDesiredPower;
        float backRightDesiredPower;
};

#endif