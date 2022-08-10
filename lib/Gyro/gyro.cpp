#include "gyro.h"

// GLOBAL VARIABLES //
Adafruit_MPU6050 my_mpu;
extern Adafruit_SSD1306 display1;

float xOff, yOff, zOff = 0; // offsets for x, y, z positions
volatile float x, y, z = 0; // will store value of x, y, z positions
int timeLastCall = millis();
float lastX, lastY, lastZ = 0;

/**
 * @brief Reads values from the gyro
 *
 * @param accel acceleration sensor event (xyz acceleration)
 * @param gyro gyro sensor event (xyz rotational velocity)
 * @param temp temperature sensor event
 * @return None
 */

void readGyro(sensors_event_t accel, sensors_event_t gyro, sensors_event_t temp)
{

  my_mpu.getEvent(&accel, &gyro, &temp);
  if (abs(gyro.gyro.x - XOFF) >= MIN_GYRO)
    x += ( 0.5 * (gyro.gyro.x + lastX )- XOFF) * CONV * (millis() - timeLastCall) / 1000;
  if (abs(gyro.gyro.y - YOFF) >= MIN_GYRO)
    y += (0.5 * (gyro.gyro.y + lastY ) - YOFF) * CONV * (millis() - timeLastCall) / 1000;
  if (abs(gyro.gyro.z - ZOFF) >= MIN_GYRO)
    z += ((0.5 * (gyro.gyro.z+ lastZ )) - ZOFF) * CONV * (millis() - timeLastCall) / 1000;
  timeLastCall = millis();
  lastX = gyro.gyro.x;
  lastY = gyro.gyro.y;
  lastZ = gyro.gyro.z;
}

/**
 * @brief Initializes and calibrates the gyro
 *
 * @param accel acceleration sensor event (xyz acceleration)
 * @param gyro gyro sensor event (xyz rotational velocity)
 * @param temp temperature sensor event
 * @return None
 */
void calibrateGyro(sensors_event_t accel, sensors_event_t gyro, sensors_event_t temp)
{
  // Try to initialize!
  if (!my_mpu.begin())
  {
    while (1)
    {
      delay(10);
    }
  }
}

/**
 * @brief integrates raw gyro values to find constant offset
 * @param accel acceleration sensor event (xyz acceleration)
 * @param gyro gyro sensor event (xyz rotational velocity)
 * @param temp temperature sensor event
 * @return determined offset in the z axis rotation
 **/

float gyroTune(sensors_event_t accel, sensors_event_t gyro, sensors_event_t temp)
{
  int i = 0;
  float xInt, yInt, zInt;
  int startTime = millis();
  while (millis() - startTime < 1000 * 30)
  {
    i++;
    my_mpu.getEvent(&accel, &gyro, &temp);
    xInt += gyro.gyro.x;
    yInt += gyro.gyro.y;
    zInt += gyro.gyro.z;
    display1.clearDisplay();
    display1.setCursor(0, 0);
    display1.print("Rotation X: ");
    display1.println(xInt / i * 1000);
    display1.print("Rotation Y: ");
    display1.println(yInt / i * 1000);
    display1.print("Rotation Z: ");
    display1.println(zInt / i * 1000);
    display1.display();
  }
  return zInt / i;
}

/**
 * @brief Sets global gyro position variables to 0
 *
 * @return None
 */
void resetGyro()
{
  x = 0;
  y = 0;
  z = 0;
}

/**
 * @brief Prints gyro information to the display
 *
 * @return None
 */
void printGyro()
{
  display1.clearDisplay();
  display1.setCursor(0, 0);
  display1.print("Rotation X: ");
  display1.println(x);
  display1.print("Rotation Y: ");
  display1.println(y);
  display1.print("Rotation Z: ");
  display1.println(z);
  display1.display();
}

void resetTimer()
{
  timeLastCall = millis();
}