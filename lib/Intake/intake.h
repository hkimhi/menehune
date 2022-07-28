#ifndef INTAKE_H
#define INTAKE_H

#include <Adafruit_SSD1306.h>

// PIN I/O
#define SERVO_POS_POT PB1       // Potentiometer to control servo closed position
#define SERVO PA8               // Pin for servo controlling intake claw
#define BUMPER_SWITCH PB11      // Front bumper for hitting pedestal
#define HALL_INPUT PB10         // Hall effect sensors to detect magnet

// VARIABLES
#define INTAKE_SERVO_OPEN_POS 0
extern Servo intakeServo;
extern bool intakeEnabled; // determines if the intake system is enabled or disabled

// FUNCTION DECLARATIONS
void printIntake();
void onHit();
void onDetectBomb();
void startIntake();
void resetClaw();

#endif