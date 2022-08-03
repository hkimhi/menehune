#include "menu.h"

Item::Item(String name, std::vector<Option> options)
{
    this->name = name;
    this->options = options;
}

Option::Option(String name, int val, int maxVal)
{
    this->name = name;
    this->val = val;
    this->maxVal = maxVal;
}

std::vector<Option> reflectanceOptions{Option("cliff ref", 150, 600), Option("line ref", 1200, 3300)};
std::vector<Option> driveOptions{};
std::vector<Option> gyroOptions{};
std::vector<Option> intakeOptions{Option("closed position", 180, 180)};

Item items[NUM_MENU_ITEMS] = {
    Item("Reflect", reflectanceOptions),
    Item("Drive", driveOptions),
    Item("Gyro", gyroOptions),
    Item("Intake", intakeOptions),
};

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
            display.println(map(pot_val, 0, 3300, 0, item.options[i].maxVal));
            // display.printf("%s: %i\n", item.options[i].name, pot_val);
        }
        else
        {
            display.setTextColor(SSD1306_WHITE);
            display.print(item.options[i].name);
            display.print(": ");
            display.println(item.options[i].val);
            // display.printf("%s: %i\n", item.options[i].name, item.options[i].value);
        }
    }

    if (!digitalRead(JOYSTICK_SWITCH))
    {
        // button pressed --> save value
        item.options[selectedOption].val = map(pot_val, 0, 3300, 0, item.options[selectedOption].maxVal);
    }
}