#include "drive.h"

// GLOBAL VARIABLES //
extern Adafruit_SSD1306 display1;

volatile float counter = 0;
volatile int ij = 0;
bool drive = false;

/**
 * @brief Initializes the drivetrain
 *
 */
void driveSetup()
{ 
  pinMode(ENC_PIN2, INPUT);
  pinMode(IR_PIN1, INPUT_ANALOG);
  pinMode(IR_PIN2, INPUT_ANALOG);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN), encCount, CHANGE);
}

/**
 * @brief Drive a motor through an H-Bridge circuit using PWM
 *
 * @param fowardPin
 * @param reversePin
 * @param power The power to drive the motor. Accepts values in range -1 to 1, inclusive
 * @return None
 */
void driveMotor(PinName fowardPin, PinName reversePin, float power)
{
  if (power > 0)
  {
    pwm_start(reversePin, 100, 0, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
    pwm_start(fowardPin, 100, (int)(32768 * power), TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  }
  else if (power < 0)
  {
    pwm_start(fowardPin, 100, 0, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
    pwm_start(reversePin, 100, (int)(32768 * abs(power)), TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
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
 * @param dir direction to turn, either +1 (left side) or 0 (right side)
 * @param accel acceleration sensor event (xyz acceleration)
 * @param gyro gyro sensor event (xyz rotational velocity)
 * @param temp temperature sensor event
 * @return None
 */
void PIDTurn(float setPoint, int dir, sensors_event_t accel, sensors_event_t gyro, sensors_event_t temp)
{
  float sat = 0.6;
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
    power = clip(power, -sat, sat);
    if (abs(error + prevError) / 2 < 0.6)
    {
      gyCo++;
    }
    if (dir == 1)
    {
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, -power * LCOMP);
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, copysign(0.20 * LCOMP, power));
    }
    else
    {
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, copysign(0.20, -power) * LCOMP);
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, power);
    }

    // printDrive(power, error, errorSum, prevError);
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
void PIDDrive(float dist, float sat, bool useIR, sensors_event_t accel, sensors_event_t gyro, sensors_event_t temp)
{
  float iSat = 100;
  float pTurn = 0.1;
  float pIR = -25;
  int error, prevError, errorSum = 0;
  float turnError, turnSet;
  float power, turnPower;
  int start = counter;
  int timeout = 0;
  drive = true;
  float setPoint = (dist / (6.28 * 3.5)) * 48;
  readGyro(accel, gyro, temp);
  turnSet = z;
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, copysign(sat, dist));
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, copysign(sat, dist));
  while ((timeout < 10))
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
  
    power += copysign(FFD, error);

    power = clip(power, -sat, sat);

    if ((prevError == error) || abs(error + prevError) <= 2)
    {
      timeout++;
    }
    prevError = error;
    delay(50);

    //Calculate Angle Correction Error
    if(!useIR){
       turnPower = (turnError * pTurn);
    }
    else{
      turnPower = (goertzel(IR_PIN1, 10, 4) - goertzel(IR_PIN2, 10, 4)) * pIR;
    }

    //Clip Turnpower to Power to prevent robot from going backwards
    turnPower = clip(turnPower, -abs(power), abs(power));

    //Apply power to motors
    driveMotor(LEFT_FOWARD, LEFT_REVERSE, (power - turnPower) * LCOMP);
    driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, power + turnPower);

    printDrive(power, error, errorSum, prevError, timeout);
  }
  //Stop Motors after Reaching destination
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
  if((digitalRead(ENC_PIN) && digitalRead(ENC_PIN2)) || (!digitalRead(ENC_PIN) && !digitalRead(ENC_PIN2))){
    counter++;
  }
  else if ((digitalRead(ENC_PIN) && !digitalRead(ENC_PIN2)) || (!digitalRead(ENC_PIN) && digitalRead(ENC_PIN2))){
    counter--;
  }
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
  display1.print("Power: ");
  display1.println(power);
  display1.printf("Error:\n  %i\n  %i\n  %i\n", error, errorSum, error - prevError);
  display1.printf("Timeout: %i\n", timeout);
  display1.display();
}

/**
 * @brief Clips a value between a given low and high value
 * 
 * @param in value to be clipped
 * @param low low value for it to be clipped to
 * @param high high value for it to be clipped to 
 * 
 * @return the value of min clamped between the low and high bounds
 * 
 */
float clip(float in, float low, float high){
  if(in < low){
    return low;
  }
  else if (in > high){
    return high;
  }
  else return in;
}

void irTurn(float sat){
  int irCount = 0;
  float power, error;
  int threshold = 0.01;
  while(irCount < 10){
    error = goertzel(IR_PIN1, 10, 4) - goertzel(IR_PIN2, 10, 4);
    power = error * PTURNIR;
    clip(power, -sat, sat);
    driveMotor(LEFT_FOWARD, LEFT_REVERSE, -power);
    driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, power);
    if(error < threshold){
      irCount++;
    }
  }
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0);
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0);
}