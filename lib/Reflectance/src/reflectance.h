#ifndef REFLECTANCE_H
#define REFLECTANCE_H

#include <Arduino.h>
#include <Adafruit_SSD1306.h>

// PIN I/O
#define REFERENCE_ONE PA_9
#define REFLECTANCE_ONE PA11
#define REFLECTANCE_TWO PA12
#define REFLECTANCE_THREE PA15

#define REFERENCE_TWO PA_10
#define REFLECTANCE_FOUR PB3
#define REFLECTANCE_FIVE PB4
#define REFLECTANCE_SIX PB5

// Variables
#define REFLECTANCE_REF_ONE_ADDR 4 // EEPROM memory address for reflectance one reference
#define REFLECTANCE_REF_TWO_ADDR 8 // EEPROM memory address for reflectance two reference
extern Adafruit_SSD1306 display1;

// Function Declarations
void printReflectance();
void setReflectanceOneReference(int);
void setReflectanceTwoReference(int);
void align();

#endif