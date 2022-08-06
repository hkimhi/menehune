#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <Servo.h>
#include <EEPROM.h>
#include "drive.h"
#include "gyro.h"
#include "intake.h"
#include "reflectance.h"
#include "menu.h"
#include "utils.h"

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
void alignRightCliff(float highPower, float lowPower, int iter);


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
  intakeServo.write(intakeServoClosedPosition);
  display1.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display2.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  attachInterrupt(digitalPinToInterrupt(BUMPER_SWITCH), onHit, FALLING);     // SWITCH_INPUT is regular high (Switches in parallel with internal pull-up)
  attachInterrupt(digitalPinToInterrupt(HALL_INPUT), onDetectBomb, FALLING); // HALL_INPUT is regular high
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
  while (shouldStart == 0)
  {
    displayMenu(display2);
    displayInfoScreen(display1);
  }
  delay(500);
  intakeServo.write(INTAKE_SERVO_OPEN_POS);
  calibrateGyro(a, g, temp);
  delay(300);

  PIDDrive(173, 0.63, false, a, g, temp); // drive up starting ramp
  resetGyro();
  PIDTurn(-17, 1, a, g, temp); // aim towards first pedestal (CW)
  prepareClaw();
  PIDDrive(16, 0.40, false, a, g, temp);  // drive at pedestal
  delay(500);                             // pick up treasure
  PIDDrive(-18, 0.45, false, a, g, temp); // reverse from pedestal
  unprepareClaw();
  PIDTurn(0, 1, a, g, temp); // turn away from pedestal (CCW)

  resetGyro();
  PIDDrive(28, 0.42, false, a, g, temp); // drive forward about to the surface edge
  PIDTurn(35, 1, a, g, temp);            // rotate CCW
  prepareClaw();
  PIDDrive(40, 0.45, false, a, g, temp); // drive forward about to the surface edge

  alignRightCliff(0.38, 0.38, 1);


  onHit(); // closes claw manually for second treasure (if not bomb)
  delay(1000);
  resetGyro();
  PIDDrive(-20, 0.42, false, a, g, temp); // drive backwards
  delay(500);
  unprepareClaw();

  // Get through arch with series of slight forward drives and turns
  PIDTurn(22.5, 0, a, g, temp);
  PIDDrive(26, 0.40, false, a, g, temp);
  PIDTurn(30, 1, a, g, temp);
  PIDDrive(5, 0.42, false, a, g, temp);
  PIDTurn(35, 1, a, g, temp);
  PIDDrive(5, 0.42, false, a, g, temp);
  PIDTurn(40, 1, a, g, temp);
  PIDDrive(5, 0.42, false, a, g, temp);
  PIDTurn(45, 1, a, g, temp);
  PIDDrive(59, 0.42, false, a, g, temp);

  //irTurn(0.5); // face IR beacon
  resetGyro();
  PIDTurn(45, 0, a, g, temp);          // turn towards third pedestal
  prepareClaw();
  delay(500);
  PIDDrive(20, 0.34, false, a, g, temp); // drive at third pedestal
  onHit();
  PIDDrive(-20, 0.34, false, a, g, temp);
  unprepareClaw();
  PIDTurn(0, 1, a, g, temp);
  PIDDrive(50, 0.43, false, a,g,temp);
  //irTurn(0.5);

  while (1) 
  {
    displayMenu(display2);
    // displayInfoScreen(display1);
    printReflectance();
  }
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

void alignRightCliff(float highPower, float lowPower, int iter) {
  // while loop to drive along right cliff edge towards the pedestal until we hit the pedestal with the bumper
  int ledgeCount = 0;
  while (getBumperState())
  {
    ledgeCount++;
    // while front bumper hasn't hit anything (i.e. until we hit the second pedestal)
    while (!digitalRead(REFLECTANCE_ONE) && getBumperState())
    {
      // drive until right wing detecting cliff
      if(ledgeCount < iter){
        driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, highPower);
        driveMotor(LEFT_FOWARD, LEFT_REVERSE, highPower);
      }
      else{
        driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, lowPower);
        driveMotor(LEFT_FOWARD, LEFT_REVERSE, lowPower);
      }
    }

    while (digitalRead(REFLECTANCE_ONE) && getBumperState())
    {
      // turn until right wing not detecting cliff
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.17);
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, -0.65);
    }

    delay(10);
    //intakeEnabled = true;
    prepareClaw();
  }
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0);
}