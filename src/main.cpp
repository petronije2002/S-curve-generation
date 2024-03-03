#include <Arduino.h>
#include "esp_heap_caps.h"

#define ACCELERATION_POINTS 100
#define CONSTANT_VELOCITY_POINTS 300
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
    Serial.println(angle[i], 3);
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
    constantVelocityPoints = totalPoints - accelerationPoints - decelerationPoints ;
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
      x[i] = currentIndex * totalTime / (totalPoints );
      float t1 = 1 / (1 + exp(-aScaled * (x[i] - c)));
      float t2 = 1 / (1 + exp(-aScaled*1.01 * (x[i] - (totalTime - c))));

      // Acceleration
      if (currentIndex < accelerationPoints)
        y[i] = fabs(t1 - t2) * targetVelocity;

      // Constant Velocity
      else if (currentIndex < accelerationPoints + constantVelocityPoints)
        y[i] = targetVelocity;

      // Deceleration
      else
        y[i] = fabs(t2 - t1) * targetVelocity;

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

float a = 10;
float c = 40;
float c2 = 0.15;

float acceleration_time = c2 *2;
float deceleration_time = acceleration_time;


// Define the sigmoid function
float sigmoid(float x, float a, float c, float c2) {
    return a / (1 + exp(-c * (x - c2)));
}


float trapezoidal_rule_integral(float x, float a, float c, float c2, int n) {
    float integral = 0;
    float h = x / n;

    for (int i = 0; i < n; i++) {
        float xi = i * h;
        float xi1 = (i + 1) * h;
        integral += (sigmoid(xi, a, c, c2) + sigmoid(xi1, a, c, c2)) * h / 2;
    }

    return integral;
}




void setup()
{
  Serial.begin(115200);
  // Example usage with a total of 120 points
  // generateVelocityProfile(2, 0.3, 2.5, TOTAL_POINTS);

  // Rest of your setup code
}

// Define the control loop
void loop() {
    float t = 0;
    float time_interval = 0.1;
    float current_angle = 0;
    float target_angle = 100;
    float max_velocity = 10;
    int n = 1000;

    while (t < acceleration_time) {
        float v_cmd = max_velocity * sigmoid(t, a, c, c2);
        current_angle += trapezoidal_rule_integral(time_interval, a, c, c2, n);
        t += time_interval;
        // set_motor_speed(v_cmd);
        // set_motor_position(current_angle);
        delay(time_interval * 1000); // Convert to milliseconds
    }

    while (t < acceleration_time + deceleration_time) {
        float v_cmd = max_velocity * sigmoid(t - acceleration_time, a, c, c2);
        current_angle += trapezoidal_rule_integral(time_interval, a, c, c2, n);
        t += time_interval;
        // set_motor_speed(v_cmd);
        // set_motor_position(current_angle);
        delay(time_interval * 1000); // Convert to milliseconds
    }

    while (current_angle < target_angle) {
        float v_cmd = max_velocity * sigmoid(current_angle / max_velocity, a, c, c2);
        current_angle += trapezoidal_rule_integral(time_interval, a, c, c2, n);
        // set_motor_speed(v_cmd);
        // set_motor_position(current_angle);
        delay(time_interval * 1000); // Convert to milliseconds
    }

    // Stop the motor when it reaches the target angle
    // set_motor_speed(0);
}

