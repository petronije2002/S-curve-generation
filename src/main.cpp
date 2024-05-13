#include <Arduino.h>
#include "MagneticEncoder.h"
#include <SPI.h>

float a = 1;
float c = 40;
float c2 = 0.15;

float angleToGo = 1;

float max_velocity = 4;
float acceleration_time = c2 * 2;
float constVelocity_time = angleToGo / max_velocity - 2 * acceleration_time;

const int csPin = 5;

SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE1);

float t = 0;

float sigmoid(float x, float a, float c, float c2)
{
  return a / (1 + exp(-c * (x - c2)));
}

float calculatedCommandVelocity(float &t, float &max_velocity)
{

  

  float v_cmd = 0;

  if (t < acceleration_time)
  {
    v_cmd = max_velocity * sigmoid(t, a, c, c2);
  }
  if (t >= acceleration_time && t < constVelocity_time + acceleration_time)
  {
    v_cmd = max_velocity;
  }
  if (t >= (constVelocity_time + acceleration_time) && t <= angleToGo / max_velocity)
  {
    v_cmd = max_velocity - max_velocity * sigmoid(t - acceleration_time - constVelocity_time, a, c, c2);
  }

  t += 0.01; // time_interval, assuming a time interval of 50 ms

  return v_cmd;
}

void setup()
{
  Serial.begin(115200);
}

void sendToSerial(float &t, float &v_cmd, float &max_velocity)
{

  Serial.print(t, 4);
  Serial.print(", ");

  Serial.print(max_velocity);
  Serial.print(", ");

  Serial.print(v_cmd, 4);

  Serial.println(); // Add a new line at the end
};

void loop()
{


  if (constVelocity_time < 0)
  {

    max_velocity = 2 * (acceleration_time+0.1) / angleToGo;

    constVelocity_time = 0;
  }

  float v_cmd = calculatedCommandVelocity(t, max_velocity);

  float currentAngle = readAngle1(csPin);

  sendToSerial(t, v_cmd, max_velocity);

  if (t > 100 / max_velocity)
  {
    t = 0; // Reset time after one complete cycle
  }

  // delay(5); // Delay to control the loop execution rate
}
