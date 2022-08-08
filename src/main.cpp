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
Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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
  display2.begin(SSD1306_SWITCHCAPVCC, 0x3D);
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

  delay(100);
}

void loop()
{
  calibrateGyro(a, g, temp);
  resetTimer();
  
  // while (shouldStart == 0)
  // {
  //   displayMenu(display2);
  //   displayInfoScreen(display1);
  // }

  
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0);

  delay(500);
  intakeServo.write(INTAKE_SERVO_OPEN_POS);
  calibrateGyro(a, g, temp);
  delay(300);

  PIDDrive(173, 0.63, false, a, g, temp); // drive up starting ramp
  prepareClaw();
  PIDTurn(-35, 1, a, g, temp);          // aim at first pedestal


  PIDDrive(16, 0.42, true, a, g, temp); // drive at first pedestal
  onHit();
  delay(500);
  PIDDrive(-14, 0.42, false, a, g, temp); // reverse from first pedestal
  unprepareClaw();
  PIDTurn(0, 1, a, g, temp);             // turn to aim forward
  PIDDrive(16, 0.42, false, a, g, temp); // drive forward
  PIDTurn(30, 0, a, g, temp);            // rotate robot in 2 steps
  PIDTurn(90, 1, a, g, temp);

  resetTimer();

  PIDDrive(48, 0.42, false, a, g, temp); // Drive straight over the chicken wire
  PIDTurn(50, 1, a, g, temp);            // turn to face for the ledge
  prepareClaw();
  alignRightCliff(0.38);
  delay(500);
  resetGyro();
  PIDDrive(-30, 0.42, false, a, g, temp); // backup robot to realign with align right cliff
  PIDTurn(-6, 1, a, g, temp);
  alignRightCliff(0.41);
  onHit();
  delay(1000);
  resetGyro();
  delay(500);
  PIDDrive(-20, 0.42, false, a, g, temp); // Back away from the pedestal
  unprepareClaw();
  delay(500);

  PIDTurn(25, 0, a, g, temp);
  PIDDrive(21, 0.39, true, a, g, temp);
  PIDTurn(90, 0, a, g, temp);            // turn towards the arch to go through
  PIDDrive(51, 0.42, false, a, g, temp); // drive through arch
  
  
  PIDTurn(180, 0, a, g, temp);           // turn towards third pedestal
  PIDDrive(-8, 0.6, false, a, g, temp);  // give space for claw to open
  prepareClaw();                         // open claw
  delay(300);
  PIDDrive(20, 0.39, true, a, g, temp); // drive at third pedestal
  onHit();                             // collect third treasure
  delay(600);

  PIDDrive(-20, 0.55, false, a, g, temp); // Back away from third pedestal
  unprepareClaw();                        // close claw for safe storage during transprt
  PIDTurn(90, 1, a, g, temp);             // aim towards IR beacon

  irTurn(0.6);
  
  PIDDrive(72, 0.43, false, a, g, temp);  // drive to position by fourth pedestal
  PIDTurn(0, 0, a, g, temp);              // turn towards fourth pedestal
  prepareClaw();                          // open claw
  PIDDrive(35, 0.34, true, a, g, temp);   // drive at fourth pedestal
  onHit();                                // collect fourth treasure
  delay(500);

  PIDDrive(-20, 0.5, false, a, g, temp); // Back away from fourth pedestal
  unprepareClaw();                       // close claw for safe storage during transport
  PIDTurn(90, 0, a, g, temp);            // move away from fourth claw towards IR beacon
  PIDDrive(11, 0.42, true, a, g, temp);  // poisiton to drop bridge
  resetGyro();

  PIDTurn(35, 1, a, g, temp);            // turn part of the way CCW to get back towards bridge
  PIDDrive(10, 0.42, false, a, g, temp); // drive forward a bit
  PIDTurn(90, 0, a, g, temp);            // turn the rest of the way CCW to get back perpendicular to cliff
  minDriveReverse();                     // backup super slowly until it sees the cliff
  PIDDrive(5, 0.42, false, a, g, temp);  // drive foward to make space for bridge

  bridgeServo.write(180); // deploy bridge

  delay(100);                            // wait for bridge to settle
  PIDDrive(19, 0.42, false, a, g, temp); // drive forward a bit

  PIDDrive(-75, 0.7, false, a, g, temp); // drive backwards very quickly to power up the bridge
  resetGyro();
  delay(2000);

  // FOR 5TH SMALL SILVER TREASURE //
  PIDTurn(-90, 0, a, g, temp);         // rotate towards 5th pedestal
  prepareClaw();                       // open claw for pickup
  PIDDrive(8, 0.5, false, a, g, temp); // drive towards 5th pedestal
  onHit();                             // pickup 5th treasure
  delay(200);
  PIDDrive(-4, 0.5, false, a, g, temp); // drive backwards towards center of platform
  unprepareClaw();

  // FOR 6TH GOLDEN TREASURE //
  PIDTurn(90, 0, a, g, temp); // turn towards end of suspension bridge

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
  PIDDrive(-8, 0.5, false, a, g, temp); // drive backwards towards platform center

  PIDTurn(-90, 0, a, g, temp); // rotate so back of robot is facing starting suspension bridge platform

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

void alignRightCliff(float power)
{
  // while loop to drive along right cliff edge towards the pedestal until we hit the pedestal with the bumper
  int turnInc = 0;
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
    }
    delay(70);
    turnInc = 0;
    resetTimer();
    while (digitalRead(REFLECTANCE_ONE) && getBumperState())
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
      readGyro(a, g, temp);
      // turn until right wing not detecting cliff
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, -0.13);
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, -0.3 - (turnInc / 128. - 0));
      prevEnc = enc;
      delay(10);
    }
    delay(30);
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
      powerInc = powerInc - 2;
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
  while (!digitalRead(REFLECTANCE_ONE) && getBumperState())
  {
    enc = counter;
    if (abs(enc - prevEnc) < 1)
    {
      powerInc++;
    }
    else
    {
      powerInc = powerInc - 2;
    }
    prevEnc = enc;
    driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, powerInc / 128.0 * dir);
    driveMotor(LEFT_FOWARD, LEFT_REVERSE, powerInc / 128.0 * dir);
    delay(20);
  }
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.0);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.0);
}