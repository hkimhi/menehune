#include "reflectance.h"
#include "drive.h"

int referenceOneDutyCycle = 0;
int referenceTwoDutyCycle = 0;

extern sensors_event_t a;
extern sensors_event_t g;
extern sensors_event_t temp;

/**
 * @brief Print reflectance information to the display
 *
 * @return None
 */
void printReflectance()
{
    display1.clearDisplay();
    display1.setCursor(0, 0);

    // display1.printf("POT: %4imV | %2i\%\n", pot_val_voltage, duty_cycle_percent);
    display1.printf("Ref one duty: %i\n", referenceOneDutyCycle);
    display1.printf("Right wing: %i\n", digitalRead(REFLECTANCE_ONE));
    display1.printf("Left wing: %i\n", digitalRead(REFLECTANCE_TWO));
    display1.printf("Back: %i\n", digitalRead(REFLECTANCE_THREE));

    display1.printf("Ref two duty: %i\n", referenceTwoDutyCycle);
    display1.printf("Front left: %i\n", digitalRead(REFLECTANCE_FOUR));
    display1.printf("Front middle: %i\n", digitalRead(REFLECTANCE_FIVE));
    display1.printf("Front right: %i\n", digitalRead(REFLECTANCE_SIX));

    display1.display();
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