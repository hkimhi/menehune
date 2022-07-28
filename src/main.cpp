#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>
#include "drive.h"
#include "gyro.h"
#include "ir_sensor.h"

// PIN I/O //
#undef LED_BUILTIN
#define LED_BUILTIN PB2
#define CLAW_PIN PA2 // intake claw servo pin

// CONSTANTS //
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // This display does not have a reset pin accessible
Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// FUNCTION DECLARATION //
void resetButton();
void openClaw(float angle);
void printGyro();

// GLOBAL VARIABLES //
Servo intakeServo;    // servo used for claw intake
sensors_event_t a;    // acceleration
sensors_event_t g;    // gyro
sensors_event_t temp; // temperature

void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  display1.begin(SSD1306_SWITCHCAPVCC, 0x3C);

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

  intakeServo.attach(CLAW_PIN);
  delay(100);
}

void loop()
{
  display1.display();
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

/**
 * @brief Opens the intake claw to a specified angle
 *
 * @param angle the angle (in degrees) to open the claw
 * @return None
 */
void openClaw(float angle)
{
  intakeServo.write(220 * angle);
}
