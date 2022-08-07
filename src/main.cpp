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
void alignRightCliff();
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
  // while (1)
  // {
  //   display1.clearDisplay();
  //   display1.setTextSize(1);
  //   display1.setTextColor(SSD1306_WHITE);
  //   display1.setCursor(0, 0);
  //   display1.println(goertzel(IR_PIN1, 25, 8) - goertzel(IR_PIN2, 25, 8));
  //   display1.println(goertzel(IR_PIN1, 25, 8));
  //   display1.println(goertzel(IR_PIN2, 25, 8));
  //   display1.display();
  // }
  
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0);
 
  delay(500);
  intakeServo.write(intakeServoClosedPosition);
  calibrateGyro(a, g, temp);
  delay(300);

  PIDDrive(173, 0.63, false, a, g, temp); // drive up starting ramp

  resetGyro();
  PIDDrive(18, 0.42, false, a, g, temp); // drive forward about to the surface edge
  PIDTurn(45, 0, a, g, temp);            // rotate CCW
  PIDDrive(33, 0.42, false, a, g, temp); // drive forward about to the surface edge
  PIDTurn(37, 1, a, g, temp);            // rotate CCW


            // rotate CCW

  alignRightCliff();


  // onHit(); // closes claw manually for second treasure (if not bomb)
  delay(1000);
  resetGyro();
  PIDDrive(-20, 0.42, false, a, g, temp); // drive backwards
  delay(500);
  // unprepareClaw();

  // Get through arch with series of slight forward drives and turns
  PIDTurn(22.5, 0, a, g, temp);
  PIDDrive(30, 0.39, false, a, g, temp);
  PIDTurn(30, 1, a, g, temp);
  PIDDrive(5, 0.39, false, a, g, temp);
  PIDTurn(35, 1, a, g, temp);
  PIDDrive(2, 0.39, false, a, g, temp);
  PIDTurn(40, 1, a, g, temp);
  PIDDrive(1, 0.39, false, a, g, temp);
  PIDTurn(45, 1, a, g, temp);
  irTurn(0.7);
  PIDDrive(48, 0.42, false, a, g, temp);

  //irTurn(0.5); // face IR beacon
  resetGyro();
 
  PIDDrive(110, 0.45, false, a,g, temp);
  PIDTurn(17.5, 1, a, g, temp);
  PIDDrive(10, 0.42, false, a, g, temp); 
  PIDTurn(45, 0, a, g, temp);
  minDriveReverse();
  PIDDrive(5, 0.42, false, a, g, temp); 

  bridgeServo.write(180);
  delay(1000);
  PIDDrive(15, 0.42, false, a, g, temp); 
  PIDDrive(-75, 0.7, false, a, g, temp); 
  delay(2000);
  PIDTurn(88, 0, a, g, temp);
  alignRightCliff();
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

void alignRightCliff() {
  // while loop to drive along right cliff edge towards the pedestal until we hit the pedestal with the bumper
  while (getBumperState())
  {
    while (!digitalRead(REFLECTANCE_ONE) && getBumperState())
    {
      // turn until right wing not detecting cliff
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.42);
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.42);
    }
    delay(20);
    while (digitalRead(REFLECTANCE_ONE) && getBumperState())
    {
      // turn until right wing not detecting cliff
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, -0.2);
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, -0.7);
    }
    delay(10);
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
    driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, -powerInc / 1024.0);
    driveMotor(LEFT_FOWARD, LEFT_REVERSE, -powerInc / 1024.0);
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
    delay(10);
  }
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.0);
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0.0);
}