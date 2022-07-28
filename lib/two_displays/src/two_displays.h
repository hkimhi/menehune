#ifndef TWO_DISPLAYS_H
#define TWO_DISPLAYS_H

#include <Adafruit_SSD1306.h>

// Pin I/O
#define JOYSTICK_SWITCH PB9
#define JOYSTICK_Y PA2
#define JOYSTICK_X PA3
#define INPUT_POT PA4


// Constants
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // This display does not have a reset pin accessible

#define NUM_MENU_ITEMS 3

// Function Declarations
void displayInfoScreen(Adafruit_SSD1306 display);
void displayMenu(Adafruit_SSD1306 display);

#endif