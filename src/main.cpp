#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>

#include "drive.h"
#include "gyro.h"
#include "ir_sensor.h"
#include "intake.h"

// PIN I/O //
#undef LED_BUILTIN
#define LED_BUILTIN PB2

// CONSTANTS //
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // This display does not have a reset pin accessible
Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

extern int intakeServoClosedPosition;
extern int intakeServoPosition;

// FUNCTION DECLARATION //
void resetButton();

// GLOBAL VARIABLES //
sensors_event_t a;    // acceleration sensor event
sensors_event_t g;    // gyro sensor event
sensors_event_t temp; // temperature sensor event

void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SERVO_POS_POT, INPUT_ANALOG);
  pinMode(BUMPER_SWITCH, INPUT_PULLUP);
  pinMode(HALL_INPUT, INPUT_PULLUP);
  pinMode(SWITCH_STATES_INPUT, INPUT_PULLUP);
  intakeServo.attach(SERVO);
  display1.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  attachInterrupt(digitalPinToInterrupt(BUMPER_SWITCH), onHit, FALLING);     // SWITCH_INPUT is regular high (Switches in parallel with internal pull-up)
  attachInterrupt(digitalPinToInterrupt(HALL_INPUT), onDetectBomb, FALLING); // HALL_INPUT is regular high
  attachInterrupt(digitalPinToInterrupt(SWITCH_STATES_INPUT), switchStates, FALLING);

  intakeServoClosedPosition = map(analogRead(SERVO_POS_POT), 0, 1023, 0, 120); // the servo has a range from 0 to 120 degrees

  calibrateGyro(a, g, temp);
  digitalWrite(LED_BUILTIN, HIGH);
  driveSetup();
  delay(2000);

  display1.clearDisplay();
  display1.setTextSize(1);
  display1.setTextColor(SSD1306_WHITE);
  display1.setCursor(0, 0);
  display1.println("MPU6050 Found!");
  display1.display();

  delay(100);
}

void loop()
{
  PIDDrive(180, a, g, temp);
  PIDTurn(-20, 1, a, g, temp);
  PIDTurn(20, 1, a, g, temp);
  PIDTurn(22.5, 0, a, g, temp);
  PIDTurn(22.5, 1, a, g, temp);
  PIDDrive(125, a, g, temp);
  PIDTurn(-22.5, 0, a, g, temp);
  PIDDrive(16, a, g, temp);
  PIDTurn(-22.5, 0, a, g, temp);
  PIDDrive(-50, a, g, temp);

  return;
}
