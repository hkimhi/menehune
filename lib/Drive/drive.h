#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <gyro.h>
#include <ir_sensor.h>
#include <math.h>

// Pin I/O
#define LEFT_FOWARD PA_7   // Left side drive, forward direction
#define LEFT_REVERSE PB_0  // Left side drive, reverse direction
#define RIGHT_FOWARD PB_8  // Right side drive, forward direction
#define RIGHT_REVERSE PB_9 // Riht side drive, reverse direction
#define ENC_PIN PB12       // Motor encoder
#define ENC_PIN2 PB13
#define IR_PIN1 PA_1
#define IR_PIN2 PA_0
#define PWM_FREQ 500

// Constants
#define P_TURN 0.9
#define I_TURN 0.01
#define D_TURN 2
#define P_DRIVE 0.25
#define I_DRIVE 0.000
#define D_DRIVE 0.25
#define FFT 0.4
#define FFD 0.02
#define LCOMP 1
#define P_TURN_DRIVE 0.4
#define D_TURN_DRIVE 0.3
extern volatile float counter;
extern volatile int ij;

#define PID_TURN_SAT_ADDR 12
#define PID_PIR_ADDR 20
#define PID_PTURNIR_ADDR 24
#define PID_DTURNIR_ADDR 28

// Variables
extern float turnSat;
extern int pIR;
extern int pTurnIR;
extern int dTurnIR;

void driveMotor(PinName fowardPin, PinName reversePin, float power);
void PIDTurn(float setPoint, int dir, sensors_event_t a, sensors_event_t g, sensors_event_t temp);
void PIDDrive(float dist, float satDr, bool useIR, sensors_event_t accel, sensors_event_t gyro, sensors_event_t temp);
void encCount();
void driveSetup();
float clip(float in, float low, float high);
void irTurn(float sat);

void setTurnSat(float);
void setPIR(int);
void setPTurnIR(int);
void setDTurnIR(int);

#endif