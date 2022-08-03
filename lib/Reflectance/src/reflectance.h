#ifndef REFLECTANCE_H
#define REFLECTANCE_H

#include <Arduino.h>
#include <Adafruit_SSD1306.h>

// PIN I/O
#define REFERENCE_ONE PA9
#define REFLECTANCE_ONE PA11
#define REFLECTANCE_TWO PA12
#define REFLECTANCE_THREE PA15

#define REFERENCE_TWO PA10
#define REFLECTANCE_FOUR PB3
#define REFLECTANCE_FIVE PB4
#define REFLECTANCE_SIX PB5

// Variables
extern Adafruit_SSD1306 display1;

// Function Declarations
void printReflectance();
void setReflectanceReference(int reference, int voltage);
void align();

#endif