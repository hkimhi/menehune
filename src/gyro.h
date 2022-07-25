#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>

#define MIN_GYRO     0.01
#define CONV 37/90.0

Adafruit_SSD1306 display_handler2;
Adafruit_MPU6050 mpu;
float xOff, yOff, zOff = 0;
volatile float x,y,z = 0;

void readGyro(sensors_event_t a, sensors_event_t g, sensors_event_t temp);
void calibGyro(sensors_event_t a, sensors_event_t g, sensors_event_t temp, Adafruit_SSD1306 display);
void resetGyro();

void readGyro(sensors_event_t a, sensors_event_t g, sensors_event_t temp){
   mpu.getEvent(&a, &g, &temp);
  if(abs(g.gyro.x + xOff) >= MIN_GYRO) x += (g.gyro.x + xOff) * CONV;
  if(abs(g.gyro.y + yOff) >= MIN_GYRO) y += (g.gyro.y + yOff) * CONV;
  if(abs(g.gyro.z + zOff) >= MIN_GYRO) z += (g.gyro.z + zOff) * CONV;
}

void calibGyro(sensors_event_t a, sensors_event_t g, sensors_event_t temp, Adafruit_SSD1306 display){
    // Try to initialize!
  pinMode(PB2, OUTPUT);
  if (!mpu.begin()) {
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  mpu.getEvent(&a, &g, &temp);
  xOff = -g.gyro.x;
  yOff = -g.gyro.y;
  zOff = -g.gyro.z;
  display_handler2 = display;
}

void resetGyro(){
  x=0;
  y=0;
  z=0;
}
