#ifndef MYSERVO_H
#define MYSERVO_H

#include <Arduino.h>

class Servos
{
private:
    int pin;
    int angle;
public:
    Servos(int pin);
    Servos();

    void initialize(int pin);
    void write(int angle);
    int read();
};

#endif