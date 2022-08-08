#include <Arduino.h>
#include <Servo.h>
#include <EEPROM.h>
#include "intake.h"

Servo intakeServo;
int intakeServoClosedPosition = 140;
extern Adafruit_SSD1306 display1;

bool intakeEnabled = false;
bool currentlySeesBomb = false;
bool hasSeenBomb = false;

/**
 * @brief Prints intake information to the display
 *
 * @param display the Adafruit_SSD1306 display to write to
 * @return None
 */
void printIntake(Adafruit_SSD1306 display)
{
  display.setCursor(0, 0);
  display.clearDisplay();
  display.printf("intakeEnabled: %i\n", intakeEnabled);
  display.printf("currentlySeesBomb: %i\n", currentlySeesBomb);
  display.printf("Closed Position: %i\n", intakeServoClosedPosition);
  display.printf("Servo Position: %i\n", intakeServo.read());
  display.printf("Bumper Switch: %i\n", digitalRead(BUMPER_SWITCH));
  display.printf("NotMagnet: %i\n", digitalRead(HALL_INPUT));
  display.display();
}

/**
 * @brief When the bumper switches activate, close the claw if the robot is in the intake state
 *
 * @return None
 */
void onHit()
{
  if (intakeEnabled && !currentlySeesBomb)
  {
    intakeServo.write(intakeServoClosedPosition);
    printIntake(display1);
  }
}

/**
 * @brief When the hall-effect sensors see a magnet, open the claw if th robot is in the intake state
 *
 * @return None
 */
void onDetectBomb()
{
  if (intakeEnabled)
  {
    intakeServo.write(INTAKE_SERVO_OPEN_POS);
    currentlySeesBomb = true;
    hasSeenBomb = true;
  }
}

/**
 * @brief Prepares the claw for the next podium by enabling intake and setting currentlySeesBomb to false
 *
 * @return None
 */
void prepareClaw()
{
  intakeServo.write(INTAKE_SERVO_OPEN_POS);
  intakeEnabled = true;
  currentlySeesBomb = false;
}

/**
 * @brief Unprepares the claw for safe transport by closing the claw and disabling intake functionality
 *
 * @return None
 */
void unprepareClaw()
{
  intakeServo.write(intakeServoClosedPosition);
  intakeEnabled = false;
}

/**
 * @brief Set the closed position for the intake servo
 *
 * @param closedPosition
 */
void setClosedPosition(int closedPosition)
{
  EEPROM.put(SERVO_CLOSED_POS_ADDR, closedPosition);
  intakeServoClosedPosition = closedPosition;
}

/**
 * @brief Returns the state of the bumper switch
 *
 * @return true - if bumper is NOT hitting anything
 * @return false  - if bumper has hit something
 */
bool getBumperState()
{
  return digitalRead(BUMPER_SWITCH);
}