#ifndef MYSERVO_H
#define MYSERVO_H

#include <Arduino.h>

class Servos
{
private:
    PinName pin;
    int angle;

public:
    Servos(PinName pin);

    void write(int angle);
    int read();
};

#endif