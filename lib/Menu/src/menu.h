#ifndef TWO_DISPLAYS_H
#define TWO_DISPLAYS_H

#include <Adafruit_SSD1306.h>
#include <WString.h>
#include <vector>
using namespace std;

// Pin I/O
#define JOYSTICK_SWITCH PC13
#define JOYSTICK_X PA2
#define JOYSTICK_Y PA3
#define INPUT_POT PA4

// Constants
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // This display does not have a reset pin accessible

#define NUM_MENU_ITEMS 3

// Function Declarations
void displayInfoScreen(Adafruit_SSD1306 display);
void displayMenu(Adafruit_SSD1306 display);

class Option
{
private:
  vector<int> mapData;

public:
  Option(String name, int val);
  void setVal(int val) { mapData[0] = val; }
  int getVal() { return mapData[0]; }

  String name;
};

class Item
{
public:
  Item(String name, vector<Option> options);

  String name;
  vector<Option> options;
};

#endif