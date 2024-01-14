#include "Arduino.h"
#include "Motor.h"
#include "PID_v1.h"
#include "MotionGenerator.h"
#include "DistanceUtiil.h"

Motor::Motor(int pin1, int pin2, Encoder* encoder, double pid_constants[]): 
    pin1(pin1),
    pin2(pin2),
    encoder(*encoder),
    reversed(false),
    input(0),
    output(0),
    setpoint(0),
    kp(pid_constants[0]),
    ki(pid_constants[1]),
    kd(pid_constants[2]),
    pid(PID(&this->input, &this->output, &this->setpoint, this->kp, this->ki, this->kd, DIRECT))
{
    this->pid.SetOutputLimits(-1, 1);
}

void Motor::forward(int speed) {
    analogWrite(pin1, speed);
    analogWrite(pin2, 0);
}

void Motor::backward(int speed) {
    analogWrite(pin1, 0);
    analogWrite(pin2, speed);
}

void Motor::set(double speed) {
    int value = (int) (speed * 255);
    if (this->reversed) {
        if (value < 0) {
            forward(value);
        }
        else {
            backward(value);
        }
    } 
    else {
        if (value < 0) {
            backward(value);
        }
        else {
            forward(value);
        }
    }
}

void Motor::setPosition(double position) {
    this->setpoint = position;
    while (abs(this->input - this->setpoint) > 0.5) {
        this->input = this->encoder.get();
        this->pid.Compute();
        this->set(this->output);
    }
}

int Motor::getPosition() {
    return this->encoder.get();
}

void Motor::update() {
    this->pid.Compute();
}

void Motor::stop() {
    analogWrite(pin1, 0);
    analogWrite(pin2, 0);
}

void Motor::setInverted(bool direction) {
    this->reversed = direction;
}
