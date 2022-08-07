#include <Arduino.h>
#include "MyServo.h"

Servos::Servos(int pin){
  this->pin = pin;
  this->angle = 0;
}

Servos::Servos(){
  this->pin = 0;
  this->angle = 0;
}

void Servos::initialize(int newPin){
  this->pin = newPin;
}

void Servos::write(int newAngle){
  this-> angle = newAngle;
  pwm_start(PinName(this->pin), 50, map(angle, 0, 180, 400, 2600), TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
}

int Servos::read(){
  return this->angle;
}