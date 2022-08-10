#ifndef TWO_DISPLAYS_H
#define TWO_DISPLAYS_H

#include <Adafruit_SSD1306.h>
#include <WString.h>
#include <vector>

// Pin I/O
#define JOYSTICK_SWITCH PC15
#define JOYSTICK_Y PA2
#define JOYSTICK_X PA3
#define INPUT_POT PA4

// Constants
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // This display does not have a reset pin accessible

#define NUM_MENU_ITEMS 5
#define NUM_MAX_OPTIONS 4

// Function Declarations
void displayInfoScreen(Adafruit_SSD1306 display);
void displayMenu(Adafruit_SSD1306 display);
void initializeMenu();

class Option
{
public:
    Option(String name, int val, int minVal, int maxVal, void (*func)(int));
    Option(String name, float val, float minVal, float maxVal, void (*func)(float));

    String name;
    bool isInt;

    int intVal;
    int intMinVal;
    int intMaxVal;
    void (*intFunc)(int);

    float floatVal;
    float floatMinVal;
    float floatMaxVal;
    void (*floatFunc)(float);
};

class Item
{
public:
    Item(String name, std::vector<Option> options);

    String name;
    std::vector<Option> options;
};

#endif