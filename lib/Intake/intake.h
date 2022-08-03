#ifndef INTAKE_H
#define INTAKE_H

#include <Servo.h>
#include <Adafruit_SSD1306.h>

// PIN I/O
#define SERVO_POS_POT PB1  // Potentiometer to control servo closed position
#define SERVO PA8          // Pin for servo controlling intake claw
#define BUMPER_SWITCH PB15 // Front bumper for hitting pedestal
#define HALL_INPUT PB14    // Hall effect sensors to detect magnet

// VARIABLES
#define INTAKE_SERVO_OPEN_POS 0
extern Servo intakeServo;
extern bool intakeEnabled; // determines if the intake system is enabled or disabled
extern bool hasSeenBomb;

// FUNCTION DECLARATIONS
void printIntake();
void onHit();
void onDetectBomb();
void resetClaw();
void setClosedPosition(int);

#endif