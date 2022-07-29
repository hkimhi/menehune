#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <gyro.h>
#include <ir_sensor.h>

// Pin I/O
#define LEFT_FOWARD PA_7 // Left side drive, forward direction
#define LEFT_REVERSE PB_0 // Left side drive, reverse direction
#define RIGHT_FOWARD PB_8  // Right side drive, forward direction
#define RIGHT_REVERSE PB_9 // Riht side drive, reverse direction
#define ENC_PIN PB12       // Motor encoder
#define IR_PIN1 PA_5
#define IR_PIN2 PA_4

// Constants
#define P_TURN 0.5
#define I_TURN 0.01
#define D_TURN 1
#define P_DRIVE 0.01
#define I_DRIVE 0.000
#define D_DRIVE 0.01
#define FFT 0.22
#define FFD 0.02
#define LCOMP 1
extern volatile float counter;

void driveMotor(PinName fowardPin, PinName reversePin, float power);
void PIDTurn(float setPoint, int dir, sensors_event_t a, sensors_event_t g, sensors_event_t temp);
void PIDDrive(float setPoint, float sat, bool useIR, sensors_event_t a, sensors_event_t g, sensors_event_t temp);
void encCount();
void driveSetup();
void printDrive(float power, int error, int errorSum, int prevError);
void printDrive(float power, int error, int errorSum, int prevError, int timeout);

#endif