#include <Arduino.h>
#include "esp_heap_caps.h"

#define ACCELERATION_POINTS 100
#define CONSTANT_VELOCITY_POINTS 20
#define DECELERATION_POINTS 100

int TOTAL_POINTS = ACCELERATION_POINTS + CONSTANT_VELOCITY_POINTS + DECELERATION_POINTS;

int NUM_POINTS_PER_CALL = 10;

void sendSerialData(float x[], float y[], float angle[], int pointsToGenerate)
{
  for (int i = 0; i < pointsToGenerate; i++)
  {
    Serial.print(x[i], 4);
    Serial.print('\t');
    Serial.print(y[i], 4);
    Serial.print('\t');
    Serial.println(angle[i], 2);
    Serial.print('\t');
  }
}


float accumulatedAngle = 0.0;


void generateVelocityProfile(float targetVelocity, float sTimeDuration, float targetAngle, int totalPoints)
{
  float totalTime = 2 * sTimeDuration + (targetAngle / targetVelocity);
  float aScaled = 40;
  float c = sTimeDuration / 2;


  float ideal_time = targetAngle / targetVelocity;


  


 float angleAtSigmoid = sTimeDuration  + (1 / aScaled) * log(1 + exp(-aScaled * (sTimeDuration - c)));

  float totalAngleAchieved = angleAtSigmoid * 2 + (totalTime - 2*sTimeDuration)*targetVelocity; 

  float scaleFactor = targetAngle / totalAngleAchieved;  


  // Calculate the number of points for each segment
  int accelerationPoints = ACCELERATION_POINTS;
  int constantVelocityPoints = CONSTANT_VELOCITY_POINTS;
  int decelerationPoints = DECELERATION_POINTS;


  // Check if the total number of points is greater than the sum of points for all segments
  if (totalPoints > accelerationPoints + constantVelocityPoints + decelerationPoints)
  {
    // Adjust the number of points for the constant velocity part to ensure the total matches
    constantVelocityPoints = totalPoints - accelerationPoints - decelerationPoints - 1;
  }

  float x[NUM_POINTS_PER_CALL];
  float y[NUM_POINTS_PER_CALL];
  float angle[NUM_POINTS_PER_CALL];

  int currentIndex = 0;
  int remainingPoints = totalPoints;


  while (remainingPoints > 0)
  {
    int pointsToGenerate = min(NUM_POINTS_PER_CALL, remainingPoints);

    // Acceleration, Constant Velocity, and Deceleration combined in one loop
    for (int i = 0; i < pointsToGenerate; i++)
    {
      x[i] = currentIndex * totalTime / (totalPoints - 1);
      float t1 = 1 / (1 + exp(-aScaled * (x[i] - c)));
      float t2 = 1 / (1 + exp(-aScaled * (x[i] - (totalTime - c))));

      // Acceleration
      if (currentIndex < accelerationPoints)
        y[i] = fabs(t1 - t2) * targetVelocity;

      // Constant Velocity
      else if (currentIndex < accelerationPoints + constantVelocityPoints)
        y[i] = targetVelocity;

      // Deceleration
      else
        y[i] = fabs(t1 - t2) * targetVelocity;

      // Calculate angle
      
      angle[i] = accumulatedAngle* scaleFactor ;


      accumulatedAngle += y[i] * (totalTime / totalPoints);
      // accumulatedAngle += y[i] * (totalTime / totalPoints) * normalizationFactor;

      currentIndex++;
    }

    sendSerialData(x, y, angle, pointsToGenerate);
    remainingPoints -= pointsToGenerate;
  }

  Serial.end(); // Close the serial port
}

void setup()
{
  Serial.begin(115200);
  // Example usage with a total of 120 points
  generateVelocityProfile(2, 0.3, 2.5, TOTAL_POINTS);

  // Rest of your setup code
}

void loop()
{
  // Your main loop codesxaxas
  //dssfdfsdfs
}
