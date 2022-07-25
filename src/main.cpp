#include <Arduino.h>
#include <Adafruit_SSD1306.h>

#define LED_BUILTIN PB2
#define REFLECTANCE_ONE PB10
#define REFLECTANCE_TWO PA10
#define REFLECTANCE_THREE PB9
#define POT PA0
#define REFERENCE_ONE PB1

#define POLL_RATE 100

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(REFLECTANCE_ONE, INPUT_PULLUP);
  pinMode(REFLECTANCE_ONE, INPUT_PULLUP);
  pinMode(REFLECTANCE_ONE, INPUT_PULLUP);
  pinMode(POT, INPUT);

  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
 
  // Displays Adafruit logo by default. call clearDisplay immediately if you don't want this.
  display_handler.display();
  
  delay(2000);

  display_handler.clearDisplay();
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(POLL_RATE/2);
  digitalWrite(LED_BUILTIN, LOW);
  delay(POLL_RATE/2);

  // int pot_val = analogRead(POT);
  int pot_val = 475;
  int duty_cycle_value = map(pot_val, 0, 1023, 0, 255); //convert raw pot input to duty cycle in range 0-255 for use in analogWrite
  int pot_val_voltage = map(pot_val, 0, 1023, 0, 3300); //convert raw pot input to mV
  int duty_cycle_percent = map(pot_val, 0, 1023, 0, 100); //convert raw pot input to duty cycle in %
  analogWrite(REFERENCE_ONE, duty_cycle_value);

  display_handler.setCursor(0,0);
  display_handler.clearDisplay();

  display_handler.printf("POT: %4imV | %2i\%\n", pot_val_voltage, duty_cycle_percent);
  display_handler.printf("Reflectance 1: %i\n", digitalRead(REFLECTANCE_ONE));
  display_handler.printf("Reflectance 2: %i\n", digitalRead(REFLECTANCE_TWO));
  display_handler.printf("Reflectance 3: %i\n", digitalRead(REFLECTANCE_THREE));

  display_handler.display();
}