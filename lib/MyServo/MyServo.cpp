#include <Arduino.h>
#include "MyServo.h"

Servos::Servos(PinName pin)
{
  this->pin = pin;
  this->angle = 0;
  pinMode(pinNametoDigitalPin(pin), OUTPUT);
}

void Servos::write(int angle)
{
  this->angle = angle;
  pwm_start(pin, 50, map(angle, 0, 180, 400, 2600), TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
}

int Servos::read()
{
  return angle;
}