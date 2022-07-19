#include <Arduino.h>
#include <Adafruit_SSD1306.h>

#define LED_BUILTIN PB2
#define REFLECTANCE PA1
#define POT PA0
#define ANALOG_OUT PB1
#define ANALOG_OUT_VAL PA7
#define COMPARATOR PB0

#define POLL_RATE 100

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(REFLECTANCE, INPUT);
  pinMode(POT, INPUT);
  pinMode(COMPARATOR, INPUT_PULLUP);
  pinMode(ANALOG_OUT_VAL, INPUT);

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

  int analog_input = analogRead(POT);
  int normalized_analog_input = map(analog_input, 0, 1023, 0, 255);
  analogWrite(ANALOG_OUT, normalized_analog_input);

  display_handler.setCursor(0,0);
  display_handler.clearDisplay();
  display_handler.print("Reflectance: "); display_handler.println(analogRead(REFLECTANCE));

  display_handler.print("POT: ");
  display_handler.print(analog_input);
  display_handler.print(" | ");
  display_handler.print(normalized_analog_input);
  display_handler.print(" | ");
  display_handler.print(map(analog_input, 0, 1023, 0, 3300));
  display_handler.println("mV");
  
  display_handler.print("Comparator in: "); display_handler.println(ANALOG_OUT_VAL);
  display_handler.print("Comparator: "); display_handler.println(digitalRead(COMPARATOR));
  display_handler.display();
}