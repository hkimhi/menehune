#include <Arduino.h>

/**
 * @brief Implements the goertzel algorithm to filter infrared
 * 
 * @param pin The pin to input infrared data
 * @param iter Number of cycles to intergrate over
 * @param k Value from 0 to 7 representing the frequency to look for (0kHz to 7kHz)
 * @return float - The power of the given frequency range
 */
float goertzel(int pin, int iter, int k)
{
  int n = 14;
  float r, i = 0;
  float w = 2.0 * 3.14 * k / n;
  float coeff = 2 * cos(w);
  float ci = sin(w);
  float s, prev, prev2 = 0;
  float buff[n];
  // measure step
  for (int z = 0; z < iter; z++)
  {
    prev = 0;
    prev2 = 0;
    s = 0;
    for (int i = 0; i < n; i++)
    {
      buff[i] = (1024 - analogRead(pin)) / 1024.0;
    }
    for (int j = 0; j < n; j++)
    {
      s = buff[j] + coeff * prev - prev2;
      prev2 = prev;
      prev = s;
    }
    r += prev2 * prev2 + prev * prev - coeff * prev * prev2;
  }

  return r;
}