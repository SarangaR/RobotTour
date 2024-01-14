#ifndef Encoder_h
#define Encoder_h

#include "Arduino.h"
#include "Motor.h"
#include "PID_v1.h"

class Encoder {
    private:
        int pin1;
        int pin2;
        int count;

    public:
        Encoder(int pin1, int pin2);
        void update();
        void reset();
        int get();
    
};
#endif  