#ifndef GYRO_H
#define GYRO_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>

#define MIN_GYRO 0.001
#define CONV (180 / 3.14)
#define XOFF 38.16/1000
#define YOFF -24.54/1000
#define ZOFF -44.90/1000

extern float xOff, yOff, zOff; // offsets for x, y, z positions
extern volatile float x, y, z; // will store value of x, y, z positions

void readGyro(sensors_event_t accel, sensors_event_t gyro, sensors_event_t temp);
void calibrateGyro(sensors_event_t accel, sensors_event_t gyro, sensors_event_t temp);
void resetGyro();
void printGyro();
void resetTimer();
float gyroTune(sensors_event_t accel, sensors_event_t gyro, sensors_event_t temp);

#endif