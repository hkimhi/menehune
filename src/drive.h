
#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <gyro.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // This display does not have a reset pin accessible
#define LEFT_FOWARD PB_10
#define LEFT_REVERSE PB_11
#define RIGHT_FOWARD PB_8
#define RIGHT_REVERSE PB_9
#define ENC_PIN PB12    
#define P_TURN 1     
#define I_TURN 0.01
#define D_TURN 1
#define P_DRIVE 0.002
#define I_DRIVE 0.000
#define D_DRIVE 0.01
#define FFT 0.22
#define FFD 0.0

volatile float counter = 0;
volatile int dir = 1;
bool turn = false;
bool drive  = false;

void driveMotor(PinName fowardPin, PinName reversePin, float power);
void PIDTurn(float setPoint, int dir, sensors_event_t a, sensors_event_t g, sensors_event_t temp);
void PIDDrive(float setPoint, sensors_event_t a, sensors_event_t g, sensors_event_t temp);
int sign(float in);
void encCount();
void driveSetup(Adafruit_SSD1306 display);

Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void driveSetup(Adafruit_SSD1306 display){
    attachInterrupt(digitalPinToInterrupt(ENC_PIN), encCount, CHANGE);
}

void driveMotor(PinName fowardPin, PinName reversePin, float power){
  if(power > 0){
    pwm_start(reversePin, 200, 0, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
    pwm_start(fowardPin, 200, (int)(32768 * power), TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  }
  else if(power < 0){
    pwm_start(fowardPin, 200, 0, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
    pwm_start(reversePin, 200, (int)(32768 * abs(power)), TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
  }
  else {
    pwm_start(reversePin, 200, 0, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
    pwm_start(fowardPin, 200, 0, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);

  }
}

/**
 * Template PID code Using Turning as an example, setpoint is in degrees where + is CCW;
 */
void PIDTurn(float setPoint, int dir, sensors_event_t a, sensors_event_t g, sensors_event_t temp){
  float sat = 0.7;
  float iSat = 100;
  float error, prevError, errorSum = 0;
  float power;
  resetGyro();
  turn = true;
  int gyCo = 0;
  while(gyCo < 10){
    readGyro(a,g,temp);
    //printGyro();
    error = setPoint - (z);
    //Proportioanl Term
    power = error * P_TURN;
    //Derivative Term
    power += (error - prevError) * D_TURN;
    //Integrating Term
    power += errorSum * I_TURN;
    power += sign(error) * FFT; 
    if(abs(errorSum) > iSat) errorSum = errorSum / abs(errorSum) * iSat;
    //integrate Errors
    errorSum += error;
    //Save Prev values for derivative
    prevError = error;
    if((power) > sat) power = sat;
    else if(power < -sat) power = -sat;
    if(abs(error + prevError)/2 < 0.6){
      gyCo++;
    }
    if(dir == 1){
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, -power);
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE,  sign(power) * 0.2);
    }
    else {
      driveMotor(LEFT_FOWARD, LEFT_REVERSE, -sign(power) * 0.2);
      driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, power);
    }
    display_handler.clearDisplay();
    display_handler.setCursor(0,0);
    display_handler.println("Power");
    display_handler.println(power);
    display_handler.println("Error");
    display_handler.println(error);
    display_handler.println(errorSum);
    display_handler.println(error - prevError);
    display_handler.display();
  }
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0);
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0);
}

/**
 * Template PID code Using Turning as an example, setpoint is in degrees where + is CCW;
 */
void PIDDrive(float dist, sensors_event_t a, sensors_event_t g, sensors_event_t temp){
  float sat = 0.33;
  float iSat = 100;
  float pTurn = 1;
  int error, prevError, errorSum = 0;
  float turnError, turnSet;
  float power, turnPower;
  int start = counter;
  int timeout = 0;
  drive = true;
  float setPoint = (dist / (6.28 * 3.5)) * 48;
  readGyro(a,g,temp);
  turnSet = z;
  while(drive && (timeout < 10)){
    
    readGyro(a,g,temp);
    error = setPoint - (counter - start);
    turnError = turnSet - z;
    //Proportioanl Term
    power = error * P_DRIVE;
    //Derivative Term
    power += (error - prevError) * D_DRIVE;
    //Integrating Term
    power += errorSum * I_DRIVE;
    power += sign(error) * FFD; 
    if(abs(errorSum) > iSat) errorSum = errorSum / abs(errorSum) * iSat;
    //integrate Errors
    errorSum += error;
    //Save Prev values for derivative
    if(error <= 0) dir = -1;
    else if(error > 0) dir = 1;
    power += sign(error) * FFD;
    if((power) > sat) power = sat;
    else if(power < -sat) power = -sat;
    if(abs(error + prevError) <= 2){
      drive = false;
    }
    if(prevError == error){
      timeout++;
    }
    prevError = error;
    delay(50);
    turnPower = (turnError * pTurn);
    if(abs(turnPower) >  abs(power)) turnPower = abs(power) * sign(turnPower);
    driveMotor(LEFT_FOWARD, LEFT_REVERSE, power - turnPower);
    driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, power + turnPower);
    display_handler.clearDisplay();
    display_handler.setCursor(0,0);
    display_handler.println("Power");
    display_handler.println(power);
    display_handler.println("Error");
    display_handler.println(error);
    display_handler.println(errorSum);
    display_handler.println(error - prevError);
    display_handler.println("Timeout");
    display_handler.println(timeout);
    display_handler.display();
    
  }
  driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0);
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0);
}

int sign(float in){
  return (0 < in) - (in < 0);
}

void encCount(){
  counter += dir;
}