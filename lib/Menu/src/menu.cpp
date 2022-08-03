#include "menu.h"
#include "reflectance.h"
#include "intake.h"
#include "drive.h"
#include "utils.h"
#include <EEPROM.h>

Item::Item(String name, std::vector<Option> options)
{
    this->name = name;
    this->options = options;
}

Option::Option(String name, int val, int maxVal, void (*func)(int))
{
    isInt = true;
    this->name = name;
    this->intVal = val;
    this->intMaxVal = maxVal;
    this->intFunc = func;
}

Option::Option(String name, float val, float maxVal, void (*func)(float))
{
    isInt = false;
    this->name = name;
    this->floatVal = val;
    this->floatMaxVal = maxVal;
    this->floatFunc = func;
}

// Globals
Item items[NUM_MENU_ITEMS] = {Item("default1", std::vector<Option>()), Item("default2", std::vector<Option>()), Item("default3", std::vector<Option>())};

void initializeMenu()
{
    extern int referenceOneDutyCycle;
    extern int referenceTwoDutyCycle;
    extern int intakeServoClosedPosition;
    extern float sat;
    extern float pTurn;
    extern int pIR;

    std::vector<Option> reflectanceOptions{Option("cliff ref", referenceOneDutyCycle, 255, setReflectanceOneReference), Option("line ref", referenceTwoDutyCycle, 255, setReflectanceTwoReference)};
    std::vector<Option> driveOptions{Option("sat", sat, 1.0, setSat), Option("pTurn", pTurn, 1, setPTurn), Option("pIR", pIR, 100, setPIR)};
    std::vector<Option> intakeOptions{Option("closed position", intakeServoClosedPosition, 180, setClosedPosition)};

    items[0] = Item("Reflect", reflectanceOptions);
    items[1] = Item("Drive", driveOptions);
    items[2] = Item("Intake", intakeOptions);
}

int selectedItem = 0;
int selectedOption = 0;
int enteredItem = -1;

// Function declarations
void enterItem(Adafruit_SSD1306 display, Item &item);

// Function Definitions
void displayMenu(Adafruit_SSD1306 display)
{
    bool isControlButtonPressed = !digitalRead(JOYSTICK_SWITCH);
    int x = analogRead(JOYSTICK_X);
    int y = analogRead(JOYSTICK_Y);

    if (y > 768)
    {
        // down
        selectedItem += 1;
        if (selectedItem >= NUM_MENU_ITEMS)
            selectedItem = NUM_MENU_ITEMS - 1;

        selectedOption += 1;
    }
    else if (y < 256)
    {
        // up
        selectedItem -= 1;
        if (selectedItem < 0)
            selectedItem = 0;

        selectedOption -= 1;
        if (selectedOption < 0)
            selectedOption = 0;
    }

    if (x > 768)
    {
        // left
        enteredItem = -1;
    }
    else if (x < 256)
    {
        // right
        enteredItem = selectedItem;
        selectedOption = 0;
    }

    if (enteredItem == -1)
    {
        // main menu
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.setTextSize(2);
        display.println("MENU");
        display.setTextSize(1);
        display.println();

        // print menu items
        for (int i = 0; i < NUM_MENU_ITEMS; i++)
        {
            if (i == selectedItem)
            {
                display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
                display.println(items[i].name);
            }
            else if (i != selectedItem)
            {
                display.setTextColor(SSD1306_WHITE);
                display.println(items[i].name);
            }
        }
    }
    else
    {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.setTextColor(SSD1306_WHITE);
        enterItem(display, items[enteredItem]);
    }

    display.display();
}

/**
 * @brief Displays info screen information
 *
 * @param display An Adafruit_SSD1306 instance which will be the display we print to
 * @return None
 */
void displayInfoScreen(Adafruit_SSD1306 display)
{
    display.setCursor(0, 0);
    display.clearDisplay();
    display.println("Display 1");

    display.printf("x: %i\n", analogRead(JOYSTICK_X));
    display.printf("y: %i\n", analogRead(JOYSTICK_Y));
    display.printf("switch: %i\n", digitalRead(JOYSTICK_SWITCH));

    display.printf("length: %i\n", EEPROM.length());

    display.display();
}

void enterItem(Adafruit_SSD1306 display, Item &item)
{
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.println(item.name);
    display.setTextSize(1);
    display.println();

    int pot_val = map(analogRead(INPUT_POT), 0, 1023, 0, 3300);

    for (int i = 0; i < item.options.size(); i++)
    {
        if (i == selectedOption)
        {
            display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
            display.print(item.options[i].name);
            display.print(": ");

            if (item.options[i].isInt)
            {
                display.println(map(pot_val, 0, 3300, 0, item.options[i].intMaxVal));
            }
            else
            {
                display.println(mapf(pot_val, 0, 3300, 0, item.options[i].floatMaxVal));
            }
        }
        else
        {
            display.setTextColor(SSD1306_WHITE);
            display.print(item.options[i].name);
            display.print(": ");

            if (item.options[i].isInt)
            {
                display.println(item.options[i].intVal);
            }
            else
            {
                display.println(item.options[i].floatVal);
            }
        }
    }

    if (!digitalRead(JOYSTICK_SWITCH))
    {
        // button pressed --> save value
        if (item.options[selectedOption].isInt)
        {
            item.options[selectedOption].intVal = map(pot_val, 0, 3300, 0, item.options[selectedOption].intMaxVal);
            item.options[selectedOption].intFunc(item.options[selectedOption].intVal);
        }
        else
        {
            item.options[selectedOption].floatVal = mapf(pot_val, 0, 3300, 0, item.options[selectedOption].floatMaxVal);
            item.options[selectedOption].floatFunc(item.options[selectedOption].floatVal);
        }
    }
}