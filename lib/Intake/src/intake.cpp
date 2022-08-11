#include <Arduino.h>
#include <Servo.h>
#include <EEPROM.h>
#include "intake.h"

extern Adafruit_SSD1306 display1;
Servo intakeServo;
int intakeServoClosedPosition = 140;

bool intakeEnabled = false;
volatile bool currentlySeesBomb = false;

/**
 * @brief Prints intake information to the display
 *
 * @return None
 */
void printIntake()
{
  display1.setCursor(0, 0);
  display1.clearDisplay();
  display1.printf("intakeEnabled: %i\n", intakeEnabled);
  display1.printf("currentlySeesBomb: %i\n", currentlySeesBomb);
  display1.printf("Closed Position: %i\n", intakeServoClosedPosition);
  display1.printf("Servo Position: %i\n", intakeServo.read());
  display1.printf("Bumper Switch: %i\n", digitalRead(BUMPER_SWITCH));
  display1.printf("NotMagnet: %i\n", digitalRead(HALL_INPUT));
  display1.display();
}

/**
 * @brief When the bumper switches activate, close the claw if the robot is in the intake state
 *
 * @return None
 */
void onHit()
{
  for (int i = 0; i < 140; i += 6)
  {
    if (intakeEnabled && digitalRead(HALL_INPUT))
    {
      // intake enabled & not detecting a magnet
      intakeServo.write(i);
      printIntake();
    }
    else
    {
      intakeServo.write(INTAKE_SERVO_OPEN_POS);
      i += 70;
    }
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
  delay(500);
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