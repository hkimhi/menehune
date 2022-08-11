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
void alignCliff(int, float);
// void alignRightCliff(float power);
void minDriveReverse();
void minDrive(int dir);
void alignBrige(float distance, float power);

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
  display1.setRotation(2);
  display2.display();

  display1.clearDisplay();
  display1.setTextSize(1);
  display1.setTextColor(SSD1306_WHITE);
  display1.setCursor(0, 0);
  display1.println("MPU6050 Found!");
  display2.setRotation(2);
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
      shouldRunOffset = 0;
      initializeMenu();
    }
    displayMenu(display2);
    displayInfoScreen(display1);
    readGyro(a, g, temp);
  }

  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0);
  resetGyro();
  delay(500);
  intakeEnabled = true;

  prepareClaw();

  PIDDrive(173, 0.7, false, true, a, g, temp); // drive up starting ramp
  PIDDrive(22, 0.42, true, false, a, g, temp);  // drive at first pedestal
  delay(200);
  onHit();
  delay(100);
  PIDDrive(-14, 0.42, false, false, a, g, temp); // reverse from first pedestal
  delay(120);
  unprepareClaw();
  PIDTurn(35, 0, a, g, temp); //aim partially towards chicken wire
  minDrive(1);
  PIDTurn(85, 1, a, g, temp);
  turnDriveSat = 0.2;
  PIDDrive(50, 0.65, false, true, a, g, temp); // cross chicken wire
  turnDriveSat = 0.6;
  if(!digitalRead(REFLECTANCE_ONE)){
    PIDTurn(75, 1, a, g, temp);
  }

  prepareClaw();
  alignCliff(1, 0.42); // align against right cliff edge
  onHit();               // pick up second idol
  delay(200);
  resetGyro();
  PIDDrive(-20, 0.42, false, false, a, g, temp); // Back away from the pedestal

  unprepareClaw();
  delay(200);

  PIDTurn(32, 0, a, g, temp);
  PIDDrive(20.7, 0.44, true, false, a, g, temp);

  PIDTurn(90, 0, a, g, temp); // turn towards the arch to go through
  if (PIDDrive(51, 0.5, true, false, a, g, temp))
  { // Try drive through arc, if fails turn and try again
    PIDTurn(80, 0, a, g, temp);
    PIDDrive(10, 0.45, true, false, a, g, temp);
    PIDTurn(90, 0, a, g, temp);
    PIDDrive(39, 0.45, true, false, a, g, temp);
  }

  PIDTurn(180, 0, a, g, temp);           // turn towards third pedestal
  PIDDrive(-10, 0.43, true, false, a, g, temp); // Back away from third pedestal

  prepareClaw(); // open claw
  delay(500);
  PIDDrive(20, 0.52, true, false, a, g, temp); // drive at third pedestal
  onHit();                             // collect third treasure
  delay(500);

  if (PIDDrive(-20, 0.67, true, false, a, g, temp))
  { // try to back away from first pedestal
    // PIDDrive(-20, 0.7, true, a, g, temp); //Try again if stuck
  }
  unprepareClaw();
  delay(300);
  // close claw for safe storage during transprt
  PIDTurn(90, 1, a, g, temp); // aim towards IR beacon

  PIDDrive(68, 0.5, false, false, a, g, temp); // drive to position by fourth pedestal
  PIDTurn(0, 0, a, g, temp);            // turn towards fourth pedestal
  PIDDrive(-12, 0.5, true, false, a, g, temp); // drive at fourth pedestal
  prepareClaw();
  delay(200);                           // open claw
  PIDDrive(38, 0.42, true, false, a, g, temp); // drive at fourth pedestal
  onHit();                              // collect fourth treasure
  delay(200);

  PIDDrive(-20, 0.5, false, false, a, g, temp); // Back away from fourth pedestal
  unprepareClaw();                       // close claw for safe storage during transport
  PIDTurn(90, 0, a, g, temp);            // move away from fourth claw towards IR beacon
  while(getBumperState()){
    driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.55);
    driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.55);
  }
  delay(500);
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0);
  delay(200);
  resetGyro();

  PIDDrive(-9, 0.42, true, false, a, g, temp); // reverse a bit
  PIDTurn(35, 1, a, g, temp);           // turn part of the way CCW to get back towards bridge
  PIDDrive(10, 0.42, true, false, a, g, temp); // drive forward a bit
  PIDTurn(90, 0, a, g, temp);           // turn the rest of the way CCW to get back perpendicular to cliff
    while(getBumperState()){
    driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.55);
    driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.55);
  }
  delay(200);
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0);
  delay(200);
  resetGyro();
  PIDDrive(-57, 0.42, true, false, a, g, temp);

  // minDriveReverse(); // backup super slowly until it sees the cliff
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.6);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.6);
  delay(50);

  PIDDrive(3, 0.42, false, false, a, g, temp); // drive foward to make space for bridge

  bridgeServo.write(180); // deploy bridge

  delay(150);                            // wait for bridge to settle
  PIDDrive(19, 0.42, false, false, a, g, temp); // drive forward a bit

  PIDDrive(-75, 0.7, false, false, a, g, temp); // drive backwards very quickly to power up the bridge
  
  PIDDrive(-25, 0.45, true, false, a, g, temp); // bump into zipline pole

  PIDDrive(8, 0.45, false, false, a, g, temp); // drive forward a bit
  delay(200);

  // PIDDrive(4, 0.5, false, a, g, temp);
  PIDTurn(90, 0, a, g, temp);
  // alignBrige(185, 0.6);
  PIDDrive(138, 0.7, false, true, a, g, temp);
  resetGyro();
  prepareClaw();
  delay(500);
  PIDTurn(25, 2, a, g, temp);
  while (getBumperState())
  {
    // while front bumper hasn't hit anything
    driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.46);
    driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.46);
  }
  intakeServo.write(intakeServoClosedPosition); // pick up golden treasure
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.0);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.0);
  intakeEnabled = false;
  PIDDrive(-7, 0.4, false, false, a, g, temp);
  // PIDTurn(0, 2, a, g, temp);
  // PIDDrive(-8, 0.4, false, false, a, g, temp);
  // PIDTurn(180, 2, a, g, temp);
  // PIDDrive(-138, 0.5, false, true, a,g,temp);
  // PIDTurn(100, 0, a, g, temp);
  // PIDDrive(15, 0.5, true, false, a,g,temp);

  while (1)
  {
    display1.clearDisplay();
    display1.setTextSize(1);
    display1.setTextColor(SSD1306_WHITE);
    display1.setCursor(0, 0);
    display1.println("MPU6050 Found!");
    display1.display();
    displayMenu(display2);
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

  EEPROM.get(GYRO_XOFF_ADDR, xOff);
  EEPROM.get(GYRO_YOFF_ADDR, yOff);
  EEPROM.get(GYRO_ZOFF_ADDR, zOff);
}

/**
 * @brief cliff alignment code
 *
 * @param side if side == 0, use left cliff detection. otherwise, right cliff
 * @param power power to drive with
 */
void alignCliff(int side, float power)
{
  // while loop to drive along right cliff edge towards the pedestal until we hit the pedestal with the bumper
  int turnInc, turnIter = 0;
  int enc, prevEnc = counter;

  int sensor = REFLECTANCE_ONE;
  if (side == 0)
  {
    sensor = REFLECTANCE_TWO;
  }

  while (getBumperState())
  {
    resetTimer();
    while (!digitalRead(sensor) && getBumperState())
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
    while (digitalRead(sensor))
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
      if (turnIter > 7)
      {
        turnInc = 100;
        turnIter = 4;
      }
      readGyro(a, g, temp);
      // turn until right wing not detecting cliff
      if (side == 1)
      {
        driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.1);
        driveMotor(LEFT_FOWARD, LEFT_REVERSE, -0.3 - (turnInc / 128. - 0));
      }
      else
      {
        driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.1);
        driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, -0.3 - (turnInc / 128. - 0));
      }

      prevEnc = enc;
      delay(10);
    }
    delay(5);
    turnIter++;
    // prepareClaw(); // open claw

    // intakeEnabled = true;
    // prepareClaw();
  }
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0);
}

void alignBrige(float distance, float powerSat)
{
  int startEnc = counter;
  float finishCount = distance * 48 / (6.28 * 3.5);
  float error, power, prevError = 0;
  int arrivedCount = 0;
  while (arrivedCount < 10)
  {
    readGyro(a, g, temp);
    if (!digitalRead(REFLECTANCE_ONE) && !digitalRead(REFLECTANCE_TWO))
    {
      error = finishCount - counter;
      power = error * 0.5;
      power += (error - prevError) * D_DRIVE;
      power = clip(power, -powerSat, powerSat);
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, power);
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, power);
      prevError = error;
      if (abs(error) < 3)
      {
        arrivedCount++;
      }
    }
    else if (digitalRead(REFLECTANCE_ONE))
    {
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.3);
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, -0.7);
    }
    else
    {
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, -0.7);
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.3);
    }
    delay(20);
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
  resetTimer();
  readGyro(a, g, temp);
  float startAngle = z;
  float angleError;
  int enc, prevEnc = counter;
  int powerInc = 0;
  while (getBumperState() && !digitalRead(REFLECTANCE_ONE))
  {
    readGyro(a, g, temp);
    angleError = z - startAngle;
    enc = counter;
    if (abs(enc - prevEnc) < 1)
    {
      powerInc++;
    }
    else if (abs(enc - prevEnc) > 1)
    {
      powerInc -= 1;
    }
    prevEnc = enc;
    driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, (0.35 + powerInc / 128.0) * dir - angleError * 0.2);
    driveMotor(LEFT_FOWARD, LEFT_REVERSE, (0.35 + powerInc / 128.0) * dir + angleError * 0.2);
    delay(30);
  }
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.0);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.0);
}