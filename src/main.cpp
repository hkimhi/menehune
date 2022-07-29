#include <Arduino.h>
#include <Wire.h>
#include "menu.h"

#undef LED_BUILTIN
#define LED_BUILTIN PB2

#define POLL_RATE 100

TwoWire Wire2(PB11, PB10);

Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire2, OLED_RESET);
Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire2, OLED_RESET);

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);
  pinMode(JOYSTICK_SWITCH, INPUT);
  pinMode(INPUT_POT, INPUT);

  display1.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display1.setTextSize(1);
  display1.setTextColor(SSD1306_WHITE);
  display1.setCursor(0, 0);

  display2.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  display2.setTextSize(1);
  display2.setTextColor(SSD1306_WHITE);
  display2.setCursor(0, 0);

  // Displays Adafruit logo by default. call clearDisplay immediately if you don't want this.
  display1.display();
  display2.display();

  delay(2000);

  display1.clearDisplay();
  display2.clearDisplay();
}

void loop()
{
  digitalWrite(LED_BUILTIN, HIGH);
  delay(POLL_RATE / 2);
  digitalWrite(LED_BUILTIN, LOW);
  delay(POLL_RATE / 2);

  displayInfoScreen(display1);
  displayMenu(display2);
}