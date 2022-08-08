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
  intakeServo.attach(SERVO);
  intakeServo.write(intakeServoClosedPosition);
  bridgeServo.attach(BRIDGE_SERVO);
  bridgeServo.write(0);
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
  calibrateGyro(a,g,temp);
  resetTimer();
  
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0);
 
  delay(500);
  intakeServo.write(intakeServoClosedPosition);
  calibrateGyro(a, g, temp);
  delay(300);

  PIDDrive(173, 0.63, true, a, g, temp); // drive up starting ramp
  PIDTurn(-35, 1, a, g, temp);
  PIDDrive(17, 0.42, true, a, g, temp); 
  PIDDrive(-17, 0.42, false, a, g, temp); 
  PIDTurn(0, 1, a, g, temp);
  PIDDrive(21, 0.42, false, a, g, temp); 
  PIDTurn(30, 0, a, g, temp);            
  PIDTurn(90, 1, a, g, temp);        

  PIDDrive(48, 0.42, false, a, g, temp); 
  PIDTurn(50, 1, a, g, temp);           
  alignRightCliff(0.38);
  delay(500);
  resetGyro();
  PIDDrive(-40, 0.42, false, a, g, temp); 
  PIDTurn(-6, 1, a, g, temp);            
  alignRightCliff(0.41);


 
  delay(1000);
  resetGyro();
  delay(500);
  PIDDrive(-20, 0.42, false, a, g, temp);
  delay(500);
  
  PIDTurn(25, 0, a, g, temp);
  PIDDrive(21, 0.39, true, a, g, temp);
  PIDTurn(90, 0, a, g, temp);
  PIDDrive(51, 0.42, false, a, g, temp);
  PIDTurn(180, 0, a, g, temp);          // turn towards third pedestal
 
  PIDDrive(20, 0.34, true, a, g, temp); // drive at third pedestal

  PIDDrive(-20, 0.55, false, a, g, temp); // Back away from third pedestal
  PIDTurn(90, 1, a, g, temp);
  PIDDrive(75, 0.43, false, a,g,temp); //Drive to position by fourth pedestal
  PIDTurn(0, 0, a, g, temp); // Turn towards fourth pedestal
  PIDDrive(-10, 0.42, true, a, g, temp); // back away to give roome for claw
  PIDDrive(35, 0.34, true, a, g, temp); // drive at third pedestal

  PIDDrive(-20, 0.5, false, a, g, temp); // Back away from third pedestal
  unprepareClaw();
  PIDTurn(90, 0, a, g, temp);
  PIDDrive(11, 0.42, true, a, g, temp);  //poisiton to drop bridge
  resetGyro();

  PIDDrive(60, 0.45, true, a,g, temp);
  PIDTurn(35, 1, a, g, temp);
  PIDDrive(10, 0.42, false, a, g, temp); 
  PIDTurn(90, 0, a, g, temp);
  minDriveReverse();
  PIDDrive(5, 0.42, false, a, g, temp); 

  bridgeServo.write(180);
  delay(1000);
  PIDDrive(19, 0.42, false, a, g, temp); 
  PIDDrive(-75, 0.7, false, a, g, temp); 
  delay(2000);
  PIDTurn(88, 0, a, g, temp);
  
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

void alignRightCliff(float power) {
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
      readGyro(a,g,temp);
    }
    delay(70);
    turnInc = 0;
    resetTimer();
    while (digitalRead(REFLECTANCE_ONE) && getBumperState())
    {
      enc = counter;
      if(abs(enc - prevEnc) < 1){
        turnInc++;
      }
      else{
        turnInc = turnInc  - 3;
      }
      readGyro(a,g,temp);
      // turn until right wing not detecting cliff
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, -0.13);
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, -0.3 - (turnInc / 128.-0));
      prevEnc = enc;
      delay(10);
    }
    delay(50);
    //intakeEnabled = true;
    prepareClaw();
  }
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0);
}

void minDriveReverse(){  
  int enc, prevEnc = counter;
  int powerInc = 0;
  while(!digitalRead(REFLECTANCE_THREE)){
    enc = counter;
    if(abs(enc - prevEnc) < 1){
      powerInc++;
    }
    else{
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

void minDrive(int dir){  
  int enc, prevEnc = counter;
  int powerInc = 0;
  while(!digitalRead(REFLECTANCE_ONE) && getBumperState()){
    enc = counter;
    if(abs(enc - prevEnc) < 1){
      powerInc++;
    }
    else{
      powerInc = powerInc  - 2;
    }
    prevEnc = enc;
    driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, powerInc / 128.0 * dir);
    driveMotor(LEFT_FOWARD, LEFT_REVERSE, powerInc / 128.0 * dir);
    delay(20);
  }
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.0);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.0);
}