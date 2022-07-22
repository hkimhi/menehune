// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>
#include <drive.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // This display does not have a reset pin accessible
#define DIO_READ_PIN PA0
#define CLAW PA_2

void resetButton();
void openClaw(float angle);

Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

bool go = false;
Servo intakeServo;
sensors_event_t a, g, temp;

void setup(void) {
  pinMode(DIO_READ_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DIO_READ_PIN), resetButton, FALLING);
 
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  calibGyro(a,g,temp, display_handler);
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

void openClaw(float angle){
  intakeServo.write(220 * angle);
}

