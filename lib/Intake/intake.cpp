#include <Arduino.h>
#include <Servo.h>
#include "intake.h"

extern Adafruit_SSD1306 display1;
Servo intakeServo;
int intakeServoClosedPosition = 120;

bool intakeEnabled = false;
bool currentlySeesBomb = false;
bool hasSeenBomb = false;

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
  if (intakeEnabled && !currentlySeesBomb)
  {
    intakeServo.write(intakeServoClosedPosition);
    printIntake();
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
 * @brief Resets the intake claw by moving it to the open position and setting the currentlySeesBomb flag to false
 *
 * @return None
 */
void resetClaw()
{
  if (intakeEnabled)
  {
    intakeServo.write(INTAKE_SERVO_OPEN_POS);
    currentlySeesBomb = false;
  }
}
