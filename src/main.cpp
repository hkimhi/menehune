#include <Arduino.h>
#include <Adafruit_SSD1306.h>

#define LED_BUILTIN PB2

#define POLL_RATE 100

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // This display does not have a reset pin accessible

Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  display1.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display1.setTextSize(1);
  display1.setTextColor(SSD1306_WHITE);
  display1.setCursor(0,0);

  display2.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  display2.setTextSize(1);
  display2.setTextColor(SSD1306_WHITE);
  display2.setCursor(0,0);
 
  // Displays Adafruit logo by default. call clearDisplay immediately if you don't want this.
  display1.display();
  display2.display();
  
  delay(2000);

  display1.clearDisplay();
  display2.clearDisplay();
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(POLL_RATE/2);
  digitalWrite(LED_BUILTIN, LOW);
  delay(POLL_RATE/2);

  display1.setCursor(0,0);
  display1.clearDisplay();
  display1.println("Display 1");
  display1.display();

  display2.setCursor(0,0);
  display2.clearDisplay();
  display2.println("Display 2");
  display2.display();
}