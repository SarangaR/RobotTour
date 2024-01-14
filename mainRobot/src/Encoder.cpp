#include "Arduino.h"
#include "Encoder.h"

Encoder::Encoder(int pin1, int pin2):
    pin1(pin1),
    pin2(pin2),
    count(0)
{}

void Encoder::update() {
    if (digitalRead(pin1) == digitalRead(pin2)) {
        count++;
    }
    else {
        count--;
    }
}

void Encoder::reset() {
    count = 0;
}

int Encoder::get() {
    return count;
}