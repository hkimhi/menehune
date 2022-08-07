#ifndef INTAKE_H
#define INTAKE_H

//#include <Servo.h>
#include <Adafruit_SSD1306.h>

// PIN I/O
#define SERVO PA8          // Pin for servo controlling intake claw
#define BUMPER_SWITCH PB15 // Front bumper for hitting pedestal
#define HALL_INPUT PB14    // Hall effect sensors to detect magnet

// VARIABLES
#define SERVO_CLOSED_POS_ADDR 0 // EEPROM memory address for servo closed position
#define INTAKE_SERVO_OPEN_POS 0
//extern Servos intakeServo;
extern bool intakeEnabled; // determines if the intake system is enabled or disabled
extern bool hasSeenBomb;

// FUNCTION DECLARATIONS
void printIntake();
void onHit();
void onDetectBomb();
void prepareClaw();
void unprepareClaw();
void setClosedPosition(int);
bool getBumperState();

#endif