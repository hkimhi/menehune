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
#define BRIDGE_SERVO PB1

// CONSTANTS //
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // This display does not have a reset pin accessible
TwoWire Wire2 = TwoWire(PB11, PB10);
Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire2, OLED_RESET);

extern int intakeServoClosedPosition;

// FUNCTION DECLARATION //
void putEEPROMDefaults();
void getEEPROMVals();
void alignRightCliff(float power);
void minDriveReverse();
void minDrive(int dir);

// GLOBAL VARIABLES //
sensors_event_t a;    // acceleration sensor event
sensors_event_t g;    // gyro sensor event
sensors_event_t temp; // temperature sensor event

Servo bridgeServo;

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
  intakeServo.attach(INTAKE_SERVO);
  intakeServo.write(intakeServoClosedPosition);
  bridgeServo.attach(BRIDGE_SERVO);
  bridgeServo.write(0);
  display1.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display2.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  // attachInterrupt(digitalPinToInterrupt(BUMPER_SWITCH), onHit, FALLING);     // SWITCH_INPUT is regular high (Switches in parallel with internal pull-up)
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
  calibrateGyro(a, g, temp);
  delay(100);
}

void loop()
{
  resetTimer();

  while (shouldStart == 0)
  {
    if (shouldRunOffset == 1)
    {
      gyroTune(a, g, temp);
      initializeMenu();
    }

    displayMenu(display2);
    displayInfoScreen(display1);
  }

  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0);
  resetGyro();
  delay(500);
  prepareClaw();

  PIDDrive(173, 0.63, false, a, g, temp); // drive up starting ramp

  while (1)
  {
    readGyro(a, g, temp);
    printGyro();
  }

  PIDDrive(16, 0.42, true, a, g, temp); // drive at first pedestal
  onHit();
  delay(100);
  PIDDrive(-14, 0.42, false, a, g, temp); // reverse from first pedestal
  delay(120);
  unprepareClaw();
  PIDTurn(35, 0, a, g, temp);
  minDrive(1);
  PIDTurn(87, 1, a, g, temp);
  PIDDrive(50, 0.42, false, a, g, temp); // cross chicken wire
  PIDTurn(77, 1, a, g, temp);

  prepareClaw();

  alignRightCliff(0.41);
  onHit();
  delay(200);
  resetGyro();
  PIDDrive(-20, 0.42, false, a, g, temp); // Back away from the pedestal

  unprepareClaw();
  delay(200);

  PIDTurn(32, 0, a, g, temp);
  PIDDrive(19, 0.40, true, a, g, temp);

  PIDTurn(90, 0, a, g, temp); // turn towards the arch to go through
  if (PIDDrive(51, 0.42, true, a, g, temp))
  { // Try drive through arc, if fails turn and try again
    PIDTurn(-10, 0, a, g, temp);
    PIDDrive(10, 0.45, true, a, g, temp);
    PIDTurn(0, 0, a, g, temp);
    PIDDrive(48, 0.45, true, a, g, temp);
  }

  PIDTurn(180, 0, a, g, temp);           // turn towards third pedestal
  PIDDrive(-10, 0.43, true, a, g, temp); // Back away from third pedestal

  prepareClaw(); // open claw
  delay(200);
  PIDDrive(20, 0.46, true, a, g, temp); // drive at third pedestal
  onHit();                              // collect third treasure
  delay(500);

  if (PIDDrive(-20, 0.67, true, a, g, temp))
  { // try to back away from first pedestal
    // PIDDrive(-20, 0.7, true, a, g, temp); //Try again if stuck
  }
  unprepareClaw();
  delay(300);
  // close claw for safe storage during transprt
  PIDTurn(90, 1, a, g, temp); // aim towards IR beacon

  PIDDrive(70, 0.43, false, a, g, temp); // drive to position by fourth pedestal
  PIDTurn(0, 0, a, g, temp);             // turn towards fourth pedestal
  prepareClaw();                         // open claw
  PIDDrive(38, 0.42, true, a, g, temp);  // drive at fourth pedestal
  onHit();                               // collect fourth treasure
  delay(200);

  PIDDrive(-20, 0.5, false, a, g, temp); // Back away from fourth pedestal
  unprepareClaw();                       // close claw for safe storage during transport
  PIDTurn(90, 0, a, g, temp);            // move away from fourth claw towards IR beacon
  PIDDrive(20, 0.65, true, a, g, temp);  // bump into IR sensor
  resetGyro();
  PIDDrive(-6, 0.42, true, a, g, temp); // reverse a bit
  PIDTurn(35, 1, a, g, temp);           // turn part of the way CCW to get back towards bridge
  PIDDrive(10, 0.42, true, a, g, temp); // drive forward a bit
  PIDTurn(90, 0, a, g, temp);           // turn the rest of the way CCW to get back perpendicular to cliff
  PIDDrive(25, 0.6, true, a, g, temp);  // drive forward a bit

  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.0);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.0);
  resetGyro();
  PIDDrive(-57, 0.42, true, a, g, temp);

  minDriveReverse(); // backup super slowly until it sees the cliff
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.6);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.6);
  delay(50);

  PIDDrive(3, 0.42, false, a, g, temp); // drive foward to make space for bridge

  bridgeServo.write(180); // deploy bridge

  delay(100);                            // wait for bridge to settle
  PIDDrive(19, 0.42, false, a, g, temp); // drive forward a bit

  PIDDrive(-75, 0.7, false, a, g, temp); // drive backwards very quickly to power up the bridge
  PIDDrive(-25, 0.35, true, a, g, temp); // bump into zipline pole

  PIDDrive(26, 0.42, false, a, g, temp); // drive forward a bit
  delay(200);

  PIDTurn(-45, 2, a, g, temp); // rotate towards 5th pedestal
  PIDDrive(-12, 0.42, true, a, g, temp);
  PIDTurn(-82, 2, a, g, temp); // rotate towards 5th pedestal
  PIDDrive(-9, 0.42, true, a, g, temp);

  prepareClaw();                        // open claw for pickup
  PIDDrive(20, 0.42, true, a, g, temp); // drive towards 5th pedestal
  onHit();                              // pickup 5th treasure
  delay(200);
  PIDDrive(-12, 0.5, true, a, g, temp); // drive backwards towards center of platform
  unprepareClaw();
  PIDTurn(-155, 2, a, g, temp);
  PIDDrive(-7, 0.5, true, a, g, temp); // drive backwards towards center of platform
  intakeServo.write(INTAKE_SERVO_OPEN_POS);
  delay(500);
  PIDDrive(20, 0.7, true, a, g, temp);
  for (int i = 0; i < 5; i++)
  {
    PIDDrive(-10, 0.7, true, a, g, temp);
    PIDDrive(12, 0.7, true, a, g, temp);
  }
  PIDDrive(-15, 0.7, true, a, g, temp);

  PIDTurn(-270, 2, a, g, temp);
  resetGyro();
  PIDDrive(150, 0.63, false, a, g, temp);
  PIDTurn(90, 2, a, g, temp);

  // FOR 6TH GOLDEN TREASURE //

  // drive up suspension bridge with cliff detection
  int suspensionBridgeLength = 150;
  int baseSpeed = 0.7;
  int shift = counter;
  int suspensionBridgeLengthCounterUnits = counter / 48 * (6.28 * 3.5) * suspensionBridgeLength;
  // float setPoint = (dist / (6.28 * 3.5)) * 48;
  while (counter < suspensionBridgeLengthCounterUnits + shift)
  {
    driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, baseSpeed);
    driveMotor(LEFT_FOWARD, LEFT_REVERSE, baseSpeed);

    if (digitalRead(REFLECTANCE_ONE))
    {
      // right wing cliff

      // recover from cliff
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.7);
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.3);
      delay(250);

      // re-align towards center
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.5);
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.9);
      delay(150);

      // drive straight forward
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, baseSpeed);
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, baseSpeed);
    }
    else if (digitalRead(REFLECTANCE_TWO))
    {
      // left wing cliff

      // recover from cliff
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.3);
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.7);
      delay(250);

      // re-align towards center
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.9);
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.5);
      delay(150);

      // drive straight forward
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, baseSpeed);
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, baseSpeed);
    }
  }
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0);

  PIDTurn(35, 0, a, g, temp);           // rotate towards 6th pedestal
  prepareClaw();                        // open claw for 6th (golden) treasure
  PIDDrive(8, 0.65, false, a, g, temp); // drive at 6th pedestal
  onHit();                              // pick up 6th treasure
  delay(500);
  PIDDrive(-8, 0.5, false, a, g, temp); // drive backwards towards platform center

  PIDTurn(-180, 0, a, g, temp); // rotate so back of robot is facing starting suspension bridge platform

  /*

  shift = counter;
  while (counter < suspensionBridgeLengthCounterUnits + shift)
  {
    driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, baseSpeed);
    driveMotor(LEFT_FOWARD, LEFT_REVERSE, baseSpeed);

    if (digitalRead(REFLECTANCE_ONE))
    {
      // right wing cliff

      // recover from cliff
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.7);
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.3);
      delay(250);

      // re-align towards center
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.5);
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.9);
      delay(150);

      // drive straight forward
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, baseSpeed);
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, baseSpeed);
    }
    else if (digitalRead(REFLECTANCE_TWO))
    {
      // left wing cliff

      // recover from cliff
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.3);
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.7);
      delay(250);

      // re-align towards center
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.9);
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.5);
      delay(150);

      // drive straight forward
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, baseSpeed);
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, baseSpeed);
    }
  }
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0);

  PIDTurn(180, 0, a, g, temp);                // rotate so front of robot is facing final "collection zone"
  prepareClaw();                              // open claw to let treasures fall out
  PIDDrive(30, 1, false, a, g, temp);         // drive towards final "collection zone"
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0); // stop both motors, hopefully some treasures fall out of bot
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0);

  // END

  while (1)
  {
    displayMenu(display2);
    // displayInfoScreen(display1);
    printReflectance();
  }*/
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

  EEPROM.get(GYRO_XOFF_ADDR, xOff);
  EEPROM.get(GYRO_YOFF_ADDR, yOff);
  EEPROM.get(GYRO_ZOFF_ADDR, zOff);
}

void alignRightCliff(float power)
{
  // while loop to drive along right cliff edge towards the pedestal until we hit the pedestal with the bumper
  int turnInc, turnIter = 0;
  int enc, prevEnc = counter;
  while (getBumperState())
  {
    resetTimer();
    while (!digitalRead(REFLECTANCE_ONE) && getBumperState())
    {
      // turn until right wing not detecting cliff
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, power);
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, power);
      readGyro(a, g, temp);
      delay(10);
    }
    delay(70);
    turnInc = 0;
    resetTimer();
    while (digitalRead(REFLECTANCE_ONE))
    {
      enc = counter;
      if (abs(enc - prevEnc) < 1)
      {
        turnInc++;
      }
      else
      {
        turnInc = turnInc - 3;
      }
      if (turnIter > 6)
      {
        turnInc = 100;
      }
      readGyro(a, g, temp);
      // turn until right wing not detecting cliff
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.1);
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, -0.3 - (turnInc / 128. - 0));
      prevEnc = enc;
      delay(10);
    }
    delay(5);
    turnIter++;

    // intakeEnabled = true;
    // prepareClaw();
  }
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0);
}

void minDriveReverse()
{
  int enc, prevEnc = counter;
  int powerInc = 0;
  while (!digitalRead(REFLECTANCE_THREE))
  {
    enc = counter;
    if (abs(enc - prevEnc) < 1)
    {
      powerInc++;
    }
    else
    {
      powerInc = powerInc - 3;
    }
    prevEnc = enc;
    driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, (-powerInc / 1024.0) - 0.15);
    driveMotor(LEFT_FOWARD, LEFT_REVERSE, (-powerInc / 1024.0) - 0.15);
    delay(10);
  }
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.2);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.2);
  delay(100);
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.0);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.0);
}

void minDrive(int dir)
{
  int enc, prevEnc = counter;
  int powerInc = 0;
  while (getBumperState() && !digitalRead(REFLECTANCE_ONE))
  {
    enc = counter;
    if (abs(enc - prevEnc) < 1)
    {
      powerInc++;
    }
    else
    {
      powerInc = powerInc - 1;
    }
    prevEnc = enc;
    driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, (0.25 + powerInc / 128.0) * dir);
    driveMotor(LEFT_FOWARD, LEFT_REVERSE, (0.25 + powerInc / 128.0) * dir);
    delay(20);
  }
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.0);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.0);
}