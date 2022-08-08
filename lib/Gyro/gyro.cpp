#include "gyro.h"

// GLOBAL VARIABLES //
Adafruit_MPU6050 my_mpu;
extern Adafruit_SSD1306 display1;

float xOff, yOff, zOff = 0; // offsets for x, y, z positions
volatile float x, y, z = 0; // will store value of x, y, z positions
int timeLastCall = millis();


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
  if (abs(gyro.gyro.x + xOff) >= MIN_GYRO)
    x += (gyro.gyro.x + xOff) * CONV * (millis() - timeLastCall) / 1000;
  if (abs(gyro.gyro.y + yOff) >= MIN_GYRO)
    y += (gyro.gyro.y + yOff) * CONV * (millis() - timeLastCall) / 1000;
  if (abs(gyro.gyro.z + zOff) >= MIN_GYRO)
    z += (gyro.gyro.z + zOff * 1.32) * CONV * (millis() - timeLastCall) / 1000;
  timeLastCall = millis();
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

  my_mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  my_mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  my_mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  my_mpu.getEvent(&accel, &gyro, &temp);
  xOff = -gyro.gyro.x;
  yOff = -gyro.gyro.y;
  zOff = -gyro.gyro.z;
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
  display1.println(my_mpu.getSampleRateDivisor());
  display1.display();

}

void resetTimer(){
    timeLastCall = millis();
}