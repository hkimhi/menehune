/*
* @brief Blink: Turns on an LED for one second then off for
* one second and then repeats.
*/

/*
We troubleshooted on my (Chris's) computer with libraries, change what needs to be changed for your computer.
*/

// #include "Arduino.h"
#include <Adafruit_SSD1306.h>
// #include <Adafruit_GFX.h>
// #include <SPI.h>
// #include <Wire.h>
#include <Servo.h>

//Set LED_BUILTIN if it is not defined by Arduino framework
#define LED_BUILTIN PB2
#define SERVO_POS_POT_INPUT PB1
#define SERVO_OUTPUT PB0
#define SWITCH_INPUT PB11
#define HALL_INPUT PB10
//Digital input to manually change states for testing
#define SWITCH_STATES_INPUT PB5


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Servo intakeServo;

int intakeServoPosition;
int intakeServoOpenPosition = 0;
int intakeServoClosedPosition;
//Variable to tell whether we are currently trying to pick up an treasure or not
int runIntake = 0;
//Variable to tell if we have sensed a bomb or not during this intake period
int clearOfBomb;

void displayInfo();
//void intake();
void hitSomething();
void seesMagnet();
void startIntake();
void switchStates();

/**
 * @brief Initialize LED pin as digital write.
 * @param none
 * @retval none
 */
void setup()
{
  //initialize pins as inputs and outputs
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SERVO_POS_POT_INPUT, INPUT_ANALOG);
  pinMode(SWITCH_INPUT, INPUT_PULLUP);
  pinMode(HALL_INPUT, INPUT_PULLUP);
  pinMode(SWITCH_STATES_INPUT, INPUT_PULLUP);
  intakeServo.attach(SERVO_OUTPUT);

  //Setup Interrupts
  attachInterrupt(digitalPinToInterrupt(SWITCH_INPUT), hitSomething, FALLING); //SWITCH_INPUT is regular high (Switches in parallel with internal pull-up)
  attachInterrupt(digitalPinToInterrupt(HALL_INPUT), seesMagnet, FALLING);     //HALL_INPUT is regular high
  attachInterrupt(digitalPinToInterrupt(SWITCH_STATES_INPUT), switchStates, FALLING);
  //attachInterrupt(runIntake, startIntake, RISING);

  //Setup display
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);

 
  // Displays Adafruit logo by default. call clearDisplay immediately if you don't want this.
  display_handler.display();
  
  //delay(1000);

  display_handler.clearDisplay();

  //Set the closed position
  intakeServoClosedPosition = map(analogRead(SERVO_POS_POT_INPUT), 0, 1023, 0, 120); //the servo has a range from 0 to 120 degrees

  //Start with the servo closed
  //intakeServo.write(intakeServoClosedPosition);
}


void loop()
{
  //startIntake();
  displayInfo();
}

void displayInfo(){
  digitalWrite(LED_BUILTIN, HIGH);
  display_handler.setCursor(0,0);
  display_handler.clearDisplay();
  display_handler.print("runIntake:");
  display_handler.println(runIntake);
  display_handler.print("clearOfBomb:");
  display_handler.println(clearOfBomb);
  display_handler.print("Closed Position: ");
  display_handler.println(intakeServoClosedPosition);
  display_handler.print("Open Position: ");
  display_handler.println(intakeServoOpenPosition);
  display_handler.print("Servo Position: ");
  display_handler.println(intakeServoPosition);
  display_handler.print("Bumper Switch: ");
  display_handler.println(digitalRead(SWITCH_INPUT));
  display_handler.print("NotMagnet: ");
  display_handler.println(digitalRead(HALL_INPUT));
  display_handler.display();
}

//Executes when the switches on the bumper sense something
//Tells the servo to close if the robot is in the intake state
void hitSomething(){
  if(runIntake && clearOfBomb){
    intakeServoPosition = intakeServoClosedPosition;
    intakeServo.write(intakeServoPosition);
  }
}

//Executes when the hall-effect sensors see a magnet
//Puts the servo in the open position if the robot is in the intake state
void seesMagnet(){
  if(runIntake){
    intakeServoPosition = intakeServoOpenPosition;
    intakeServo.write(intakeServoPosition);
    clearOfBomb = 0;
  }
}

void switchStates(){
  if(runIntake){
    runIntake = 0;
  }
  else if(!runIntake){
    runIntake = 1;
    intakeServoPosition = intakeServoOpenPosition;
    intakeServo.write(intakeServoPosition);
    clearOfBomb = 1;
  }
}