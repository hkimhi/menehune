#include "drive.h"

// GLOBAL VARIABLES //
extern Adafruit_SSD1306 display1;

volatile float counter = 0;
volatile int dir = 1;
bool drive = false;

/**
 * @brief Initializes the drivetrain
 *
 */
void driveSetup()
{
  attachInterrupt(digitalPinToInterrupt(ENC_PIN), encCount, CHANGE);
}

/**
 * @brief Drive a motor through an H-Bridge circuit using PWM
 *
 * @param fowardPin
 * @param reversePin
 * @param power The power to drive the motor. Accepts values in range 0 to 1, inclusive
 * @return None
 */
void driveMotor(PinName fowardPin, PinName reversePin, float power)
{
  if (power > 0)
  {
    pwm_start(reversePin, 100, 0, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
    pwm_start(fowardPin, 100, (int)(32768 * power * 2), TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  }
  else if (power < 0)
  {
    pwm_start(fowardPin, 100, 0, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
    pwm_start(reversePin, 100, (int)(32768 * abs(power) * 2), TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  }
  else
  {
    pwm_start(reversePin, 100, 0, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
    pwm_start(fowardPin, 100, 0, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  }
}

/**
 * @brief PID code using turning, setpoint is in degrees where + is CCW
 *
 * @param setPoint target angle
 * @param dir direction to turn, either +1 or -1
 * @param accel acceleration sensor event (xyz acceleration)
 * @param gyro gyro sensor event (xyz rotational velocity)
 * @param temp temperature sensor event
 * @return None
 */
void PIDTurn(float setPoint, int dir, sensors_event_t accel, sensors_event_t gyro, sensors_event_t temp)
{
  float sat = 0.5;
  float iSat = 100;
  float error, prevError, errorSum = 0;
  float power;
  resetGyro();
  int gyCo = 0;
  while (gyCo < 10)
  {
    readGyro(accel, gyro, temp);
    // printGyro();
    error = setPoint - (z);
    // Proportioanl Term
    power = error * P_TURN;
    // Derivative Term
    power += (error - prevError) * D_TURN;
    // Integrating Term
    power += errorSum * I_TURN;
    power += copysign(FFT, error);
    if (abs(errorSum) > iSat)
      errorSum = errorSum / abs(errorSum) * iSat;
    // integrate Errors
    errorSum += error;
    // Save Prev values for derivative
    prevError = error;
    if ((power) > sat)
    {
      power = sat;
    }
    else if (power < -sat)
    {
      power = -sat;
    }
    if (abs(error + prevError) / 2 < 0.6)
    {
      gyCo++;
    }
    if (dir == 1)
    {
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, -power * LCOMP);
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, copysign(0.2 * LCOMP, power));
    }
    else
    {
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, copysign(0.2, -power) * LCOMP);
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, power);
    }

    printDrive(power, error, errorSum, prevError);
  }
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0);
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0);
}

/**
 * @brief PID drive with setpoint in centimeters, + is forward and - is backward
 *
 * @param dist target distance
 * @param accel acceleration sensor event (xyz acceleration)
 * @param gyro gyro sensor event (xyz rotational velocity)
 * @param temp temperature sensor event
 * @return None
 */
void PIDDrive(float dist, bool useIR, sensors_event_t accel, sensors_event_t gyro, sensors_event_t temp)
{
  float sat = 0.35;
  float iSat = 100;
  float pTurn = 0.5;
  float pIR = 10;
  int error, prevError, errorSum = 0;
  float turnError, turnSet;
  float power, turnPower;
  int start = counter;
  int timeout = 0;
  drive = true;
  float setPoint = (dist / (6.28 * 3.5)) * 48;
  readGyro(accel, gyro, temp);
  turnSet = z;
  while (drive && (timeout < 10))
  {

    readGyro(accel, gyro, temp);
    error = setPoint - (counter - start);
    turnError = turnSet - z;
    // Proportioanl Term
    power = error * P_DRIVE;
    // Derivative Term
    power += (error - prevError) * D_DRIVE;
    // Integrating Term
    power += errorSum * I_DRIVE;
    power += copysign(FFD, error);
    if (abs(errorSum) > iSat)
      errorSum = errorSum / abs(errorSum) * iSat;
    // integrate Errors
    errorSum += error;
    // Save Prev values for derivative
    if (error <= 0)
      dir = -1;
    else if (error > 0)
      dir = 1;
    power += copysign(FFD, error);
    if ((power) > sat)
      power = sat;
    else if (power < -sat)
      power = -sat;
    if (abs(error + prevError) <= 2)
    {
      drive = false;
    }
    if (prevError == error)
    {
      timeout++;
    }
    prevError = error;
    delay(50);
    if(!useIR){
       turnPower = (turnError * pTurn);
    }
    else{
      turnPower = (goertzel(IR_PIN1, 10, 4) - goertzel(IR_PIN2, 10, 4)) * pIR;
    }
    if (abs(turnPower) > abs(power))
    {
      turnPower = copysign(power, turnPower);
    }

    driveMotor(LEFT_FOWARD, LEFT_REVERSE, (power - turnPower) * LCOMP);
    driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, power + turnPower);

    printDrive(power, error, errorSum, prevError, timeout);
  }

  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0);
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0);
}

/**
 * @brief Interrupt function for the encoder.
 *
 * Sets the encoder counter value based on direction
 *
 * @return None
 */
void encCount()
{
  counter += dir;
}

/**
 * @brief Prints drive information to the display
 *uu
 * @return None
 */
void printDrive(float power, int error, int errorSum, int prevError)
{
  display1.clearDisplay();
  display1.setCursor(0, 0);
  display1.print("Power :");
  display1.println(power);
  display1.printf("Error:\n  %i\n  %i\n  %i\n", error, errorSum, error - prevError);
  display1.display();
}

/**
 * @brief Prints drive information to the display
 *
 * @return None
 */
void printDrive(float power, int error, int errorSum, int prevError, int timeout)
{
  display1.clearDisplay();
  display1.setCursor(0, 0);
  // display1.printf("Power: %f\n", power);
  display1.print("Power: ");
  display1.println(power);
  display1.printf("Error:\n  %i\n  %i\n  %i\n", error, errorSum, error - prevError);
  display1.printf("Timeout: %i\n", timeout);
  display1.display();
}