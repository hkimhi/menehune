#include "two_displays.h"

const char *options[NUM_MENU_ITEMS] = {
    " Reflectance",
    " Encoders",
    " Menu 3",
};

int selectedItem = 0;
int enteredItem = -1;

void displayMenu(Adafruit_SSD1306 display)
{
    bool isControlButtonPressed = !digitalRead(JOYSTICK_SWITCH);
    int x = analogRead(JOYSTICK_X);
    int y = analogRead(JOYSTICK_Y);

    if (x < 256)
    {
        // down
        selectedItem += 1;
        if (selectedItem >= NUM_MENU_ITEMS)
            selectedItem = NUM_MENU_ITEMS - 1;
    }
    else if (x > 768)
    {
        // up
        selectedItem -= 1;
        if (selectedItem < 0)
            selectedItem = 0;
    }

    if (y < 256)
    {
        // left
        enteredItem = -1;
    }
    else if (y > 768)
    {
        // right
        enteredItem = selectedItem;
    }

    if (enteredItem == -1)
    {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.setTextSize(2);
        display.println("MENU");
        display.setTextSize(1);
        display.println();
        for (int i = 0; i < NUM_MENU_ITEMS; i++)
        {
            if (i == selectedItem)
            {
                display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
                display.println(options[i]);
            }
            else if (i != selectedItem)
            {
                display.setTextColor(SSD1306_WHITE);
                display.println(options[i]);
            }
        }
    }
    else
    {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.setTextColor(SSD1306_WHITE);
        switch (enteredItem)
        {
        case 0:
        {
            display.setTextSize(2);
            display.println("Reflect");
            display.println();
            display.setTextSize(1);

            int tempSelectedItem = 0;

            int tempCliffRef = 0; // TODO: equal to global cliff ref
            int tempLineRef = 0;  // TODO: equal to global line ref

            const char *tempOptions[NUM_MENU_ITEMS] = {
                " Cliff ref: %dmV\n",
                " Line ref: %dmV\n"};

            int *tempArgs[NUM_MENU_ITEMS] = {
                &tempCliffRef,
                &tempLineRef};

            int pot_val = map(analogRead(INPUT_POT), 0, 1023, 0, 3300);

            for (int i = 0; i < 2; i++)
            {
                if (i == selectedItem)
                {
                    *tempArgs[i] = pot_val;

                    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
                    display.printf(tempOptions[i], *tempArgs[i]);
                }
                else if (i != selectedItem)
                {
                    display.setTextColor(SSD1306_WHITE);
                    display.printf(tempOptions[i], *tempArgs[i]);
                }
            }

            break;
        }
        case 1:
            display.println("Eneterd encoders");
            break;
        case 2:
            display.println("Entered item 3");
            break;
        default:
            display.println("DEFAULT");
        }
    }

    display.display();
}

void displayInfoScreen(Adafruit_SSD1306 display)
{
    display.setCursor(0, 0);
    display.clearDisplay();
    display.println("Display 1");
    display.display();
}