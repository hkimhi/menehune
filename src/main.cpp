// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <math.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // This display does not have a reset pin accessible
#define MIN_GYRO     0.01
#define DIO_READ_PIN PA0
#define ENC_PIN PA1    
#define LEFT_FOWARD PA_6
#define LEFT_REVERSE PA_7
#define RIGHT_FOWARD PB_0
#define RIGHT_REVERSE PB_1
#define XOFF 0.03 //Calibated Offset Compensation
#define YOFF -0.10
#define ZOFF 0.01
#define CONV 1
#define P_TURN 0.3
#define I_TURN 0.000
#define D_TURN -0.0
#define P_DRIVE 0.1
#define I_DRIVE 0.0
#define D_DRIVE -0.0
#define FF 0.2

void resetButton();
void encCount();
void printDisp();
void resetGyro();
void readGyro(sensors_event_t a, sensors_event_t g, sensors_event_t temp);
void printGyro();
void driveMotor(PinName fowardPin, PinName reversePin, float power);
void PIDTurn(float setPoint, sensors_event_t a, sensors_event_t g, sensors_event_t temp);
void PIDDrive(float setPoint, sensors_event_t a, sensors_event_t g, sensors_event_t temp);
int sign(float in);

Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MPU6050 mpu;
volatile float x,y,z = 0;
volatile float counter = 0;
volatile int dir = 1;
float vx, vy, vz, dx,dy,dz = 0;
bool turn = false;
bool drive  = false;
 

void setup(void) {
  pinMode(DIO_READ_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DIO_READ_PIN), resetButton, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN), encCount, CHANGE);
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  Serial.println("Adafruit MPU6050 test!");
  // Displays Adafruit logo by default. call clearDisplay immediately if you don't want this.
  display_handler.display();
  delay(2000);
  // Displays "Hello world!" on the screen
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);

  // Try to initialize!
  if (!mpu.begin()) {
    display_handler.clearDisplay();
    display_handler.setCursor(0,0);
    display_handler.println("Failed to find MPU6050 chip");
    display_handler.display();

    while (1) {
      delay(10);
    }
  }
  display_handler.clearDisplay();
  display_handler.setCursor(0,0);
  display_handler.println("MPU6050 Found!");
  display_handler.display();

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  

  delay(100);
}

void loop() {
 
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  readGyro(a,g,temp);
  
  printGyro();
  PIDDrive(8.5 * 48,a,g,temp);
  PIDTurn(45, a,g,temp);
  PIDDrive(3.8 * 48,a,g,temp);
  PIDTurn(45, a,g,temp);
  PIDDrive(5 * 48,a,g,temp);

  while(1){}
  //PIDTurn(30, a,g,temp);
  //PIDTurn(30, a,g,temp);
}

void resetButton(){
  drive = !drive;
  turn = !turn;
}

void encCount(){
  counter += dir;
}

void readGyro(sensors_event_t a, sensors_event_t g, sensors_event_t temp){
   mpu.getEvent(&a, &g, &temp);
  if(abs(g.gyro.x + XOFF) >= MIN_GYRO) x += g.gyro.x + XOFF;
  if(abs(g.gyro.y + YOFF) >= MIN_GYRO) y += g.gyro.y + YOFF;
  if(abs(g.gyro.z + ZOFF) >= MIN_GYRO) z += g.gyro.z + ZOFF;
}

void printGyro(){
  display_handler.clearDisplay();
  display_handler.setCursor(0,0);
  display_handler.println("Rotation X:");
  display_handler.println(x * CONV);
  display_handler.println("Rotation Y:");
  display_handler.println(y * CONV);
  display_handler.println("Rotation z:");
  display_handler.println(z * CONV);
  display_handler.display();
  delay(50);
}

void printDisp(){
  display_handler.setCursor(0,0);
  display_handler.println("Disp X:");
  display_handler.println(vx);
  display_handler.println("Disp Y:");
  display_handler.println(vy);
  display_handler.println("Disp z:");
  display_handler.println(vz);
  display_handler.display();
  delay(50);
}

void resetGyro(){
  x=0;
  y=0;
  z=0;
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
void PIDTurn(float setPoint, sensors_event_t a, sensors_event_t g, sensors_event_t temp){
  float sat = 0.5;
  float iSat = 100;
  float error, prevError, errorSum = 0;
  float power;
  resetGyro();
  turn = true;
  while(turn){
    display_handler.clearDisplay();
    display_handler.setCursor(0,0);

    readGyro(a,g,temp);
    //printGyro();
    error = setPoint - (z * CONV);
    //Proportioanl Term
    power = error * P_TURN;
    //Derivative Term
    power += (error - prevError) * D_TURN;
    //Integrating Term
    power += errorSum * I_TURN;
    if(abs(errorSum) > iSat) errorSum = errorSum / abs(errorSum) * iSat;
    //integrate Errors
    errorSum += error;
    //Save Prev values for derivative
    prevError = error;
    if((power) > sat) power = sat;
    else if(power < -sat) power = -sat;
    if(abs(error + prevError) < 1){
      turn = false;
    }
    driveMotor(LEFT_FOWARD, LEFT_REVERSE, -power);
    driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, power);
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
void PIDDrive(float setPoint, sensors_event_t a, sensors_event_t g, sensors_event_t temp){
  float sat = 0.35;
  float iSat = 100;
  float error, prevError, errorSum = 0;
  float power;
  int start = counter;
  drive = true;
  while(drive){
    display_handler.clearDisplay();
    display_handler.setCursor(0,0);

    error = setPoint - (counter - start);
    //Proportioanl Term
    power = error * P_DRIVE;
    //Derivative Term
    power += (error - prevError) * D_DRIVE;
    //Integrating Term
    power += errorSum * I_DRIVE;
    power += sign(error) * FF; 
    if(abs(errorSum) > iSat) errorSum = errorSum / abs(errorSum) * iSat;
    //integrate Errors
    errorSum += error;
    //Save Prev values for derivative
    prevError = error;
    if(error <= 0) dir = -1;
    else if(error > 0) dir = 1;
    power += sign(error) * FF;
    if((power) > sat) power = sat;
    else if(power < -sat) power = -sat;
    if(error + prevError < 1){
      drive = false;
    }
    driveMotor(LEFT_FOWARD, LEFT_REVERSE, power);
    driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, power);
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

int sign(float in){
  return (0 < in) - (in < 0);
}

