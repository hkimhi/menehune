#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <math.h>
#include <Servo.h>

#include "drive.h"
#include "gyro.h"
#include "ir_sensor.h"
#include "intake.h"
#include "menu.h"
#include "reflectance.h"

// PIN I/O //
#undef LED_BUILTIN
#define LED_BUILTIN PB2

// CONSTANTS //
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // This display does not have a reset pin accessible
Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

extern int intakeServoClosedPosition;

// FUNCTION DECLARATION //
void resetButton();

// GLOBAL VARIABLES //
sensors_event_t a;    // acceleration sensor event
sensors_event_t g;    // gyro sensor event
sensors_event_t temp; // temperature sensor event

void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);
  pinMode(JOYSTICK_SWITCH, INPUT);
  pinMode(INPUT_POT, INPUT);

  pinMode(REFLECTANCE_ONE, INPUT_PULLUP);
  pinMode(REFLECTANCE_TWO, INPUT_PULLUP);
  pinMode(REFLECTANCE_THREE, INPUT_PULLUP);
  pinMode(REFLECTANCE_FOUR, INPUT_PULLUP);
  pinMode(REFLECTANCE_FIVE, INPUT_PULLUP);
  pinMode(REFLECTANCE_SIX, INPUT_PULLUP);
  setReflectanceOneReference(1500);
  setReflectanceTwoReference(150);

  pinMode(SERVO_POS_POT, INPUT_ANALOG);
  pinMode(BUMPER_SWITCH, INPUT_PULLUP);
  pinMode(HALL_INPUT, INPUT_PULLUP);
  intakeServo.attach(SERVO);
  intakeServo.write(INTAKE_SERVO_OPEN_POS);
  // intakeServo.write(intakeServoClosedPosition);*/
  display1.begin(SSD1306_SWITCHCAPVCC, 0x3C);
<<<<<<< HEAD
=======
  display2.begin(SSD1306_SWITCHCAPVCC, 0x3D);

>>>>>>> 48df4d8084bafc17d556d843b4d486a310ebb393
  attachInterrupt(digitalPinToInterrupt(BUMPER_SWITCH), onHit, FALLING);     // SWITCH_INPUT is regular high (Switches in parallel with internal pull-up)
  attachInterrupt(digitalPinToInterrupt(HALL_INPUT), onDetectBomb, FALLING); // HALL_INPUT is regular high
  calibrateGyro(a, g, temp);
  digitalWrite(LED_BUILTIN, HIGH);
  driveSetup();
  delay(2000);

  display2.setTextSize(1);
  display2.setTextColor(SSD1306_WHITE);
  display2.setCursor(0, 0);
  display2.display();

  display1.clearDisplay();
  display1.setTextSize(1);
  display1.setTextColor(SSD1306_WHITE);
  display1.setCursor(0, 0);
  display1.println("MPU6050 Found!");
  display1.display();

  delay(100);

}
void loop()
{
<<<<<<< HEAD
  /*
  

  printIntake();
  intakeEnabled = true;
  PIDDrive(176, 0.37, false, a, g, temp);
  resetIntake();
  PIDTurn(-15, 1, a, g, temp); //first pedestal
  PIDDrive(20, 0.27, false, a, g, temp);
  PIDDrive(-20, 0.30, false, a, g, temp);
  PIDTurn(60, 0, a, g, temp); //first pedestal
  resetIntake();
  PIDDrive(77, 0.37, false, a, g, temp);
  PIDTurn(30, 1, a, g, temp); //first pedestal
  PIDDrive(40, 0.27, false, a, g, temp);
  PIDTurn(15, 1, a, g, temp); //first pedestal
  PIDDrive(50, 0.37, false, a, g, temp);
  */
  
  PIDDrive(200, 0.37, true, a, g, temp);

  while (1)
  {
    display1.clearDisplay();
    display1.setTextSize(1);
    display1.setTextColor(SSD1306_WHITE);
    display1.setCursor(0, 0);
    float start = millis();
    for(int i = 0; i < 1000; i++){
      analogRead(PA0);
    }
    display1.println((goertzel(IR_PIN1, 10, 4) - goertzel(IR_PIN2, 10, 4)) * 1000);
    display1.println("Encoder");
    display1.println(counter);
    display1.println(ij);
    display1.display();
  }
=======
  displayInfoScreen(display1);
  displayMenu(display2);
  // //intakeOff();
  // printIntake();
  // PIDDrive(180, false, a, g, temp);
  // PIDTurn(-20, 1, a, g, temp);
  // resetIntake();
  // PIDDrive(30, false, a, g, temp);
  // PIDTurn(20, 1, a, g, temp);
  // //intakeOff();
  // PIDTurn(22.5, 0, a, g, temp);
  // PIDTurn(22.5, 1, a, g, temp);
  // PIDDrive(125, false, a, g, temp);
  // PIDTurn(-22.5, 0, a, g, temp);
  // resetIntake();
  // PIDDrive(30, false, a, g, temp);
  // PIDTurn(-22.5, 0, a, g, temp);
  // PIDDrive(-50, false, a, g, temp);

  // while (1)
  // {
  //   delay(5000);
  //   resetIntake();
  // }
>>>>>>> 48df4d8084bafc17d556d843b4d486a310ebb393
}
