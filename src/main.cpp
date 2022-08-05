#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>
#include <EEPROM.h>
#include "drive.h"
#include "gyro.h"
#include "ir_sensor.h"
#include "intake.h"
#include "reflectance.h"
#include "menu.h"

// PIN I/O //
#undef LED_BUILTIN
#define LED_BUILTIN PB2

// CONSTANTS //
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // This display does not have a reset pin accessible
Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

extern int intakeServoClosedPosition;

// FUNCTION DECLARATION //
void putEEPROMDefaults();
void getEEPROMVals();

// GLOBAL VARIABLES //
sensors_event_t a;    // acceleration sensor event
sensors_event_t g;    // gyro sensor event
sensors_event_t temp; // temperature sensor event

void setup(void)
{
  // putEEPROMDefaults();
  getEEPROMVals();
  initializeMenu();
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);
  pinMode(JOYSTICK_SWITCH, INPUT);
  pinMode(INPUT_POT, INPUT);

  pinMode(REFLECTANCE_ONE, INPUT_PULLUP);
  pinMode(REFLECTANCE_TWO, INPUT_PULLUP);
  pinMode(REFLECTANCE_THREE, INPUT_PULLUP);
  pinMode(REFLECTANCE_FOUR, INPUT_PULLUP);
  pinMode(REFLECTANCE_FIVE, INPUT_PULLUP);
  pinMode(REFLECTANCE_SIX, INPUT_PULLUP);

  pinMode(SERVO_POS_POT, INPUT_ANALOG);
  pinMode(BUMPER_SWITCH, INPUT_PULLUP);
  pinMode(HALL_INPUT, INPUT_PULLUP);
  intakeServo.attach(SERVO);
  intakeServo.write(INTAKE_SERVO_OPEN_POS);
  display1.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display2.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  attachInterrupt(digitalPinToInterrupt(BUMPER_SWITCH), onHit, FALLING);     // SWITCH_INPUT is regular high (Switches in parallel with internal pull-up)
  attachInterrupt(digitalPinToInterrupt(HALL_INPUT), onDetectBomb, FALLING); // HALL_INPUT is regular high
  calibrateGyro(a, g, temp);
  digitalWrite(LED_BUILTIN, HIGH);
  driveSetup();
  delay(2000);

  display2.setTextSize(1);
  display2.setTextColor(SSD1306_WHITE);
  display2.setCursor(0, 0);
  display2.display();

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
  PIDDrive(175, 0.50, false, a, g, temp); // drive up starting ramp
  resetGyro();
  PIDTurn(-17, 1, a, g, temp); // aim towards first pedestal (CW)
  intakeEnabled = true;
  PIDDrive(20, 0.24, false, a, g, temp);  // drive at pedestal
  delay(500);                             // pick up treasure
  PIDDrive(-20, 0.32, false, a, g, temp); // reverse from pedestal
  intakeEnabled = false;
  PIDTurn(0, 1, a, g, temp); // turn away from pedestal (CCW)

  resetGyro();
  PIDDrive(31, 0.30, false, a, g, temp); // drive forward about to the surface edge
  PIDTurn(27, 1, a, g, temp);            // rotate CCW

  while(isBumper()){
    while (!digitalRead(REFLECTANCE_ONE) &&  isBumper())
    {
      // drive until right wing detecting cliff
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.3);
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.3);
    }

    while (digitalRead(REFLECTANCE_ONE) && isBumper())
    {
      // turn until right wing not detecting cliff
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.15);
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, -0.8);
    }

    delay(25);
    intakeEnabled = true;
    resetClaw();
  }
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0);
  PIDDrive(10, 0.1, false, a, g, temp);
  intakeEnabled = true;
  resetClaw();
  onHit();
  delay(500);
  resetGyro();
  PIDDrive(-20, 0.30, false, a, g, temp);
  onHit();
  intakeEnabled = false;
  PIDTurn(22.5, 0, a, g, temp); // first pedestal
  PIDDrive(30, 0.30, false, a, g, temp);
  PIDTurn(30, 1, a, g, temp); // first pedestal
  PIDDrive(10, 0.25, false, a, g, temp);
  PIDTurn(35, 1, a, g, temp); // first pedestal
  PIDDrive(10, 0.25, false, a, g, temp);
  PIDTurn(40, 1, a, g, temp); // first pedestal
  PIDDrive(10, 0.25, false, a, g, temp);
  PIDTurn(45, 1, a, g, temp); // first pedestal
  PIDDrive(30, 0.25, false, a, g, temp);

  irTurn(0.5);

  /*PIDDrive(20, 0.30, false, a, g, temp);
  PIDTurn(90, 1, a, g, temp); //first pedestal
  irTurn(0.5);
  resetGyro();
  PIDDrive(40, 0.27, true, a, g, temp);*/
  while (1)
  {
    displayMenu(display2);
    //displayInfoScreen(display1);
    display1.clearDisplay();
    display1.setTextSize(1);
    display1.setTextColor(SSD1306_WHITE);
    display1.setCursor(0, 0);
    display1.println(digitalRead(REFLECTANCE_ONE));
    display1.display();  }
}

void putEEPROMDefaults()
{
  EEPROM.put(SERVO_CLOSED_POS_ADDR, 140);
  EEPROM.put(REFLECTANCE_REF_ONE_ADDR, 150);
  EEPROM.put(REFLECTANCE_REF_TWO_ADDR, 150);
  EEPROM.put(PID_TURN_SAT_ADDR, 0.83);
  EEPROM.put(PID_PIR_ADDR, 25);
  EEPROM.put(PID_PTURNIR_ADDR, 50);
  EEPROM.put(PID_DTURNIR_ADDR, 1000);
}

void getEEPROMVals()
{
  EEPROM.get(SERVO_CLOSED_POS_ADDR, intakeServoClosedPosition);

  int refOne;
  int refTwo;
  EEPROM.get(REFLECTANCE_REF_ONE_ADDR, refOne);
  EEPROM.get(REFLECTANCE_REF_TWO_ADDR, refTwo);
  setReflectanceOneReference(refOne);
  setReflectanceTwoReference(refTwo);

  EEPROM.get(PID_TURN_SAT_ADDR, turnSat);
  EEPROM.get(PID_PIR_ADDR, pIR);
  EEPROM.get(PID_PTURNIR_ADDR, pTurnIR);
  EEPROM.get(PID_DTURNIR_ADDR, dTurnIR);
}