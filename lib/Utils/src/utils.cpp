#include "utils.h"

int shouldStart = 0;
int shouldRunOffset = 0;

/**
 * @brief Provides functionality like the standard map() function but for floats
 * 
 * @param input input value
 * @param inFrom lower bound of input value
 * @param inTo upper bound of input value
 * @param outFrom lower bound of output value
 * @param outTo upper bound of output value
 * @return float - mapped float value
 */
float mapf(float input, float inFrom, float inTo, float outFrom, float outTo) {
    float f = (input - inFrom) / (inTo - inFrom) * (outTo - outFrom) + outFrom;
    return f;
}

/**
 * @brief Determines if globally the robot should go or not
 * 
 * @param val the value to set shouldStart to (1 for true, 0 for false)
 * @return None
 */
void setStart(int val) {
    shouldStart = val;
}

void setOffset(int val) {
    shouldRunOffset = val;
}