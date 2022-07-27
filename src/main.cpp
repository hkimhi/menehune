#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#define LED_BUILTIN PB2

#define POLL_RATE 100

#define JOYSTICK_SWITCH PB9
#define JOYSTICK_Y PA2
#define JOYSTICK_X PA3
#define POT_INPUT PA4

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // This display does not have a reset pin accessible

#define NUM_MENU_ITEMS 3

int selectedItem = 0;
int enteredItem = -1;

const char *options[NUM_MENU_ITEMS] = {
  " Reflectance",
  " Encoders",
  " Menu 3",
};

TwoWire Wire2(PB11, PB10);

Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire2, OLED_RESET);
Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire2, OLED_RESET);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);
  pinMode(JOYSTICK_SWITCH, INPUT);
  pinMode(POT_INPUT, INPUT);

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

  bool isControlButtonPressed = !digitalRead(JOYSTICK_SWITCH);
  int x = analogRead(JOYSTICK_X);
  int y = analogRead(JOYSTICK_Y);

  display1.setCursor(0,0);
  display1.clearDisplay();
  display1.println("Display 1");
  display1.println(isControlButtonPressed);
  // display1.display();

  if(x < 256) {
    //down
    selectedItem += 1;
    if(selectedItem >= NUM_MENU_ITEMS) selectedItem = NUM_MENU_ITEMS-1;
  } else if (x > 768) {
    //up
    selectedItem -= 1;
    if(selectedItem < 0) selectedItem = 0;
  }

  if(y < 256) {
    //left
    enteredItem = -1;
  } else if (y > 768) {
    //right
    enteredItem = selectedItem;
  }

  if (enteredItem == -1) {
    display2.clearDisplay();
    display2.setTextSize(1);
    display2.setTextColor(SSD1306_WHITE);
    display2.setCursor(0, 0);
    display2.setTextSize(2);
    display2.println("MENU");
    display2.setTextSize(1);
    display2.println();
    for (int i = 0; i < NUM_MENU_ITEMS; i++) {
      if (i == selectedItem) {
        display2.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        display2.println(options[i]);
      } else if (i != selectedItem) {
        display2.setTextColor(SSD1306_WHITE);
        display2.println(options[i]);
      }
    }
  }
  else {
    display1.clearDisplay();
    display1.setCursor(0,0);
    display1.setTextColor(SSD1306_WHITE);
    switch(enteredItem){
      case 0: {
        display1.setTextSize(2);
        display1.println("Reflect");
        display1.println();
        display1.setTextSize(1);

        int tempSelectedItem = 0;

        int tempCliffRef = 0; //TODO: equal to global cliff ref
        int tempLineRef = 0; //TODO: equal to global line ref

        const char *tempOptions[NUM_MENU_ITEMS] = {
          " Cliff ref: %dmV\n",
          " Line ref: %dmV\n"
        };
        
        int *tempArgs[NUM_MENU_ITEMS] = {
          &tempCliffRef,
          &tempLineRef
        };

        int pot_val = map(analogRead(POT_INPUT), 0, 1023, 0, 3300);

        for(int i = 0; i < 2; i++) {
          if (i == selectedItem) {
            *tempArgs[i] = pot_val;

            display1.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
            display1.printf(tempOptions[i], *tempArgs[i]);
          } else if (i != selectedItem) {
            display1.setTextColor(SSD1306_WHITE);
            display1.printf(tempOptions[i], *tempArgs[i]);
          }
        }
        
        break;
      }
      case 1:
        display2.println("Eneterd encoders");
        break;
      case 2:
        display2.println("Entered item 3");
        break;
      default:
        display2.println("DEFAULT");
    }
  }

  display1.display();
  display2.display();
}