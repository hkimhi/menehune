#ifndef GYRO_H
#define GYRO_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>

#define MIN_GYRO 0.01
#define CONV 37 / 90.0

extern Adafruit_SSD1306 display1;

extern float xOff, yOff, zOff; // offsets for x, y, z positions
extern volatile float x, y, z; // will store value of x, y, z positions

void readGyro(sensors_event_t accel, sensors_event_t gyro, sensors_event_t temp);
void calibrateGyro(sensors_event_t accel, sensors_event_t gyro, sensors_event_t temp);
void resetGyro();
void printGyro();

#endif