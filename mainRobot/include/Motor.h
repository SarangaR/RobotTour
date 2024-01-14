#ifndef Motor_h
#define Motor_h

#include "PID_v1.h"
#include "Encoder.h"

class Motor {
    private:
        int pin1;
        int pin2;
        bool reversed;
        
        double input;
        double output;
        double setpoint;

        double kp;
        double ki;
        double kd;

        Encoder encoder;

        PID pid;

        void forward(int speed);
        void backward(int speed);
    
    public:
        Motor(int pin1, int pin2, Encoder* encoder, double pid_constants[]);
        void set(double speed);
        void setPosition(double position);
        void update();
        int getPosition();
        void reverse();
        void stop();
        void setInverted(bool direction);

};
#endif
