#include "reflectance.h"

int referenceOneDutyCycle = 475;
int referenceTwoDutyCycle = 475;

/**
 * @brief Print reflectance information to the display
 *
 * @return None
 */
void printReflectance()
{
    display1.setCursor(0, 0);
    display1.clearDisplay();

    // display1.printf("POT: %4imV | %2i\%\n", pot_val_voltage, duty_cycle_percent);
    display1.printf("Reflectance 1: %i\n", digitalRead(REFLECTANCE_ONE));
    display1.printf("Reflectance 2: %i\n", digitalRead(REFLECTANCE_TWO));
    display1.printf("Reflectance 3: %i\n", digitalRead(REFLECTANCE_THREE));
}

/**
 * @brief Set the reflectance reference voltage value
 * 
 * @param reference int, 1 for first set of reflectance sensor, 2 for second set
 * @param voltage value from 0 to 3300mv representing reference voltage to set
 * @return None
 */
void setReflectanceReference(int reference, int voltage)
{
    int duty_cycle = map(voltage, 0, 3300, 0, 255);

    if (reference == 1)
    {
        referenceOneDutyCycle = duty_cycle;
        analogWrite(REFERENCE_ONE, duty_cycle);
    }
    else
    {
        referenceTwoDutyCycle = duty_cycle;
        analogWrite(REFERENCE_TWO, duty_cycle);
    }
}