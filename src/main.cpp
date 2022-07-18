#include <Arduino.h>
#include <Adafruit_SSD1306.h>

#define LED_BUILTIN PB2
#define IO_POT PA0
#define ANALOG_OUT PA8

#define POLL_RATE 100

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(IO_POT, INPUT);

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

  int input = analogRead(IO_POT);
  int normalized_input = map(input, 0, 1023, 0, 255);

  display_handler.setCursor(0,0);
  display_handler.clearDisplay();
  display_handler.print("POT input: ");
  display_handler.println(input);
  display_handler.print("normalized input: ");
  display_handler.println(normalized_input);
  display_handler.display();

  analogWrite(ANALOG_OUT, normalized_input);

}