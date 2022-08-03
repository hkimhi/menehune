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
  intakeServo.attach(SERVO);
  intakeServo.write(INTAKE_SERVO_OPEN_POS);
  // intakeServo.write(intakeServoClosedPosition);*/
  display1.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  attachInterrupt(digitalPinToInterrupt(BUMPER_SWITCH), onHit, FALLING);     // SWITCH_INPUT is regular high (Switches in parallel with internal pull-up)
  attachInterrupt(digitalPinToInterrupt(HALL_INPUT), onDetectBomb, FALLING); // HALL_INPUT is regular high
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
  /*
  

  printIntake();
  intakeEnabled = true;
  PIDDrive(176, 0.37, false, a, g, temp);
  resetIntake();
  PIDTurn(-15, 1, a, g, temp); //first pedestal
  PIDDrive(20, 0.27, false, a, g, temp);
  PIDDrive(-20, 0.30, false, a, g, temp);
  PIDTurn(60, 0, a, g, temp); //first pedestal
  resetIntake();
  PIDDrive(77, 0.37, false, a, g, temp);
  PIDTurn(30, 1, a, g, temp); //first pedestal
  PIDDrive(40, 0.27, false, a, g, temp);
  PIDTurn(15, 1, a, g, temp); //first pedestal
  PIDDrive(50, 0.37, false, a, g, temp);
  */
  
  PIDDrive(200, 0.37, true, a, g, temp);

  while (1)
  {
    display1.clearDisplay();
    display1.setTextSize(1);
    display1.setTextColor(SSD1306_WHITE);
    display1.setCursor(0, 0);
    float start = millis();
    for(int i = 0; i < 1000; i++){
      analogRead(PA0);
    }
    display1.println((goertzel(IR_PIN1, 10, 4) - goertzel(IR_PIN2, 10, 4)) * 1000);
    display1.println("Encoder");
    display1.println(counter);
    display1.println(ij);
    display1.display();
  }
}
