#include <Arduino.h>
#include "MyServo.h"

Servos::Servos(String name, PinName pin){
  this->name = name;
  this->pin = pin;
}

void Servos::write(int newAngle){
  this-> angle = newAngle;
  pwm_start(this->pin, 50, map(angle, 0, 180, 400, 2600), TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
}

int Servos::read(){
  return this->angle;
}