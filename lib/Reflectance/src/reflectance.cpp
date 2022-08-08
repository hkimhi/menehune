#include "reflectance.h"
#include "drive.h"
#include <EEPROM.h>

int referenceOneDutyCycle;
int referenceTwoDutyCycle;

extern sensors_event_t a;
extern sensors_event_t g;
extern sensors_event_t temp;

/**
 * @brief Print reflectance information to the display
 *
 * @param display the Adafruit_SSD1306 display to write to
 * @return None
 */
void printReflectance(Adafruit_SSD1306 display)
{
    display.clearDisplay();
    display.setCursor(0, 0);

    // display1.printf("POT: %4imV | %2i\%\n", pot_val_voltage, duty_cycle_percent);
    display.printf("Ref one duty: %i\n", referenceOneDutyCycle);
    display.printf("Right wing: %i\n", digitalRead(REFLECTANCE_ONE));
    display.printf("Left wing: %i\n", digitalRead(REFLECTANCE_TWO));
    display.printf("Back: %i\n", digitalRead(REFLECTANCE_THREE));

    display.printf("Ref two duty: %i\n", referenceTwoDutyCycle);
    display.printf("Front left: %i\n", digitalRead(REFLECTANCE_FOUR));
    display.printf("Front middle: %i\n", digitalRead(REFLECTANCE_FIVE));
    display.printf("Front right: %i\n", digitalRead(REFLECTANCE_SIX));

    display.display();
}

/**
 * @brief Set the reflectance one reference duty cycle
 *
 * @param voltage value from 0 to 255 representing duty cycle to set
 * @return None
 */
void setReflectanceOneReference(int duty_cycle)
{
    referenceOneDutyCycle = duty_cycle;
    EEPROM.put(REFLECTANCE_REF_ONE_ADDR, referenceOneDutyCycle);
    analogWrite(REFERENCE_ONE, duty_cycle);
}

/**
 * @brief Set the reflectance two reference duty cycle
 *
 * @param voltage value from 0 to 255 representing duty cycle to set
 * @return None
 */
void setReflectanceTwoReference(int duty_cycle)
{
    referenceTwoDutyCycle = duty_cycle;
    EEPROM.put(REFLECTANCE_REF_TWO_ADDR, referenceTwoDutyCycle);
    analogWrite(REFERENCE_TWO, duty_cycle);
}

/**
 * @brief uses the front reflectance sensors to align the robot to the black tape line
 *
 * @return None
 */
void align()
{
    int power = 0.1;

    if (digitalRead(REFLECTANCE_FOUR))
    {
        // left is on line
        driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, power);
        driveMotor(LEFT_FOWARD, LEFT_REVERSE, -power);
    }
    else if (digitalRead(REFLECTANCE_SIX))
    {
        // right is on line
        driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, -power);
        driveMotor(LEFT_FOWARD, LEFT_REVERSE, power);
    }
    else
    {
        driveMotor(RIGHT_FOWARD, RIGHT_REVERSE, 0);
        driveMotor(LEFT_FOWARD, LEFT_REVERSE, 0);
    }
}