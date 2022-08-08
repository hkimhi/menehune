#include "gyro.h"

// GLOBAL VARIABLES //
Adafruit_MPU6050 my_mpu;

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
    z += (gyro.gyro.z + zOff) * CONV * (millis() - timeLastCall) / 1000;
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
  my_mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
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
 * @param display the Adafruit_SSD1306 display to write to
 * @return None
 */
void printGyro(Adafruit_SSD1306 display)
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Rotation X: ");
  display.println(x);
  display.print("Rotation Y: ");
  display.println(y);
  display.print("Rotation Z: ");
  display.println(z);
  display.println(my_mpu.getSampleRateDivisor());
  display.display();

}

void resetTimer(){
    timeLastCall = millis();
}