// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>
#include <drive.h>

#define DIO_READ_PIN PA0
#define LED_BULTIN PB2
#define CLAW PA_2

void resetButton();
void openClaw(float angle);
void printGyro();



bool go = false;
Servo intakeServo;
sensors_event_t a, g, temp;

void setup(void) {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DIO_READ_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DIO_READ_PIN), resetButton, FALLING);
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  calibGyro(a,g,temp, display_handler);
  digitalWrite(PB2, HIGH);
  driveSetup(display_handler);
  delay(2000);

  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.println("MPU6050 Found!");
  display_handler.display();

  
  intakeServo.attach(PA2);
  delay(100);
}

void loop() {
  digitalWrite(PB2, HIGH);
  /* Get new sensor events with the readings */
  
  readGyro(a,g,temp);
  printGyro();
  driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0.45);
  delay(250);
  //driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0);
  printGyro();

  digitalWrite(PB2, LOW);
  delay(250);

  /*PIDDrive(180, a,g,temp);
  PIDTurn(-20,1, a,g,temp);
  PIDTurn(20,1, a,g,temp);
  PIDTurn(22.5,0, a,g,temp);
  PIDTurn(22.5,1, a,g,temp);
  PIDDrive(125, a,g,temp);
  PIDTurn(-22.5,0, a,g,temp);
  PIDDrive(16, a,g,temp);
  PIDTurn(-22.5,0, a,g,temp);
  PIDDrive(-50, a,g,temp);
  
  
  while(1){}*/
}

void resetButton(){
  go = true;
}

void openClaw(float angle){
  intakeServo.write(220 * angle);
}

void printGyro(){
  display_handler.clearDisplay();
  display_handler.setCursor(0,0);
  display_handler.println("Rotation X:");
  display_handler.println(x);
  display_handler.println("Rotation Y:");
  display_handler.println(y );
  display_handler.println("Rotation z:");
  display_handler.println(z);
  display_handler.display();
  delay(50);
}
