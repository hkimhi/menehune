#include <Arduino.h>

//#ifndef MYSERVO_H
#define MYSERVO_H

class Servos
{
private:
    String name;
    PinName pin;
    int angle;
public:
    Servos(String Name, PinName pin);

    void write(int angle);
    int read();
};