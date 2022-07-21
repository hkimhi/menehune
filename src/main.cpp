// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>

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
#define CLAW PA_2
#define XOFF 0.03 //Calibated Offset Compensation
#define YOFF -0.10
#define ZOFF 0.01
#define CONV 1
#define P_TURN 1     
#define I_TURN 0.01
#define D_TURN 1
#define P_DRIVE 0.002
#define I_DRIVE 0.000
#define D_DRIVE 0.01
#define FFT 0.22
#define FFD 0.0


void resetButton();
void encCount();
void printDisp();
void resetGyro();
void readGyro(sensors_event_t a, sensors_event_t g, sensors_event_t temp);
void printGyro();
void driveMotor(PinName fowardPin, PinName reversePin, float power);
void PIDTurn(float setPoint, int dir, sensors_event_t a, sensors_event_t g, sensors_event_t temp);
void PIDDrive(float setPoint, sensors_event_t a, sensors_event_t g, sensors_event_t temp);
int sign(float in);
void calibGyro(sensors_event_t a, sensors_event_t g, sensors_event_t temp);
void openClaw(float angle);

Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MPU6050 mpu;
volatile float x,y,z = 0;
volatile float counter = 0;
float xOff, yOff, zOff = 0;
volatile int dir = 1;
float vx, vy, vz, dx,dy,dz = 0;
bool turn = false;
bool drive  = false;
bool go = false;
Servo intakeServo;
sensors_event_t a, g, temp;

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
  calibGyro(a,g,temp);
  intakeServo.attach(PA2);
  delay(100);
}

void loop() {
 
  /* Get new sensor events with the readings */
  
  readGyro(a,g,temp);
  
  printGyro(); 
  PIDDrive(180, a,g,temp);
  PIDTurn(-20,1, a,g,temp);
  PIDTurn(20,1, a,g,temp);
  PIDTurn(22.5,0, a,g,temp);
  PIDTurn(22.5,1, a,g,temp);
  PIDDrive(125, a,g,temp);
  PIDTurn(-22.5,0, a,g,temp);
  PIDDrive(16, a,g,temp);
  PIDTurn(-22.5,0, a,g,temp);
  PIDDrive(-50, a,g,temp);
  
  while(1){}
}

void resetButton(){
  go = true;
}

void encCount(){
  counter += dir;
}

void readGyro(sensors_event_t a, sensors_event_t g, sensors_event_t temp){
   mpu.getEvent(&a, &g, &temp);
  if(abs(g.gyro.x + xOff) >= MIN_GYRO) x += g.gyro.x + xOff;
  if(abs(g.gyro.y + yOff) >= MIN_GYRO) y += g.gyro.y + yOff;
  if(abs(g.gyro.z + zOff) >= MIN_GYRO) z += g.gyro.z + zOff;
}

void calibGyro(sensors_event_t a, sensors_event_t g, sensors_event_t temp){
  mpu.getEvent(&a, &g, &temp);
  xOff = -g.gyro.x;
  yOff = -g.gyro.y;
  zOff = -g.gyro.z;
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
void PIDTurn(float setPoint, int dir, sensors_event_t a, sensors_event_t g, sensors_event_t temp){
  float sat = 0.7;
  float iSat = 100;
  float error, prevError, errorSum = 0;
  float power;
  resetGyro();
  turn = true;
  int gyCo = 0;
  while(gyCo < 10){
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
  float error, prevError, errorSum = 0;
  float turnError, turnSet;
  float power, turnPower;
  int start = counter;
  int timeout = 0;
  drive = true;
  float setPoint = (dist / (6.28 * 3.5)) * 48;
  readGyro(a,g,temp);
  turnSet = z;
  while(drive && (timeout < 10)){
    display_handler.clearDisplay();
    display_handler.setCursor(0,0);
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

void openClaw(float angle){
  intakeServo.write(220 * angle);
}

