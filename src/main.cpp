#include <Arduino.h>
#include "MagneticEncoder.h"
#include <SPI.h>
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/mcpwm.h"
#include <vector>


// #include "esp32-hal-cpu.h" // Include the ESP32 SDK header for CPU related functions

const int LO1 = 14; // Pin used for PWM output
const int HO1 = 12; // Pin used for PWM output

const int LO2 = 26; // Pin used for PWM output
const int HO2 = 27; // Pin used for PWM output

const int LO3 = 33; // Pin used for PWM output
const int HO3 = 25;

const int csPin = 5;    // pin used for SPI encoder selection
const int EN_GATE = 15; // enable

bool shouldRead = false;
float currentPosition = 1.234;

#define PHASE_DELAY_1 (double)2.094395102 // 120°
#define PHASE_DELAY_2 (double)4.188790205 // 240°

struct TableEntry
{
  float angle;
  float velocity;
};

float Kp = 0.5;     // Proportional gain
float Ki = 0.1;     // Integral gain
float integral = 0; // Integral term accumulator

float Kp_angle = 0.5;     // Proportional gain for angle control
float Ki_angle = 0.1;     // Integral gain for angle control
float integral_angle = 0; // Integral term accumulator for angle control
// float prevError_angle = 0; // Previous error for derivative term (not used in this example)

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 5
const float MAX_POSITION = 359.9 * PI / 180;
const float MIN_POSITION = 0.0;
int POLE_PAIRS = 21;

const float freqCarr = 19200;            // Carrier freq.
float freqMod = 50.0;                    // Modulation Frequency Hz
int amplitude = 300;                     // max counter/timer value
int sampleNum = int(freqCarr / freqMod); //
int i;

float a = 1;
float c = 40;
float c2 = 0.15;

// float angleToGo = 1;

// float max_velocity = 4;

float acceleration_time = c2 * 2;
float deceleration_time = acceleration_time;
float constVelocity_time = 0;
float constAngle = 0;

float angleToGo = 0;
float commanded_velocity = 0;
float total_time = 0;

int num_values = 25;

int num_accel_segments = 10;

int num_const_segments = 5;

struct AngleVelocityPairTable
{
  float angle;
  float velocity;
};
// AngleVelocityPairTable lookupTable[num_values];

// AngleVelocityPairTable* lookupTable;


AngleVelocityPairTable* lookupTable = nullptr;



struct AngleVelocityPair
{
  std::vector<float> angle_values;
  std::vector<float> velocity_values;

  // Constructor to initialize the vectors with predefined sizes

  AngleVelocityPair(int angle_size, int velocity_size) : angle_values(angle_size), velocity_values(velocity_size) {}
};


AngleVelocityPair result(1,1);

float calculatedCommandVelocity(float &t, float &max_velocity);
float calculateDesiredPositionAtT(float &current_time, float prevVel, float &step_size_ad);





// AngleVelocityPair 
void generateAngleAndVelocityValues(float angleToGo, float commanded_velocity)
{

  float start_time = micros();

  total_time = angleToGo / commanded_velocity;

  constVelocity_time = total_time - 2 * acceleration_time;

  // Serial.print(" Const velocity: ");
  // Serial.print(constVelocity_time);
  // Serial.println();


  if (constVelocity_time <= 0)
  {
    num_values= 2 * num_accel_segments;

    commanded_velocity = 2 * (acceleration_time) / angleToGo;

    total_time = angleToGo / commanded_velocity;

    constVelocity_time = 0;

    num_const_segments = 0;

    Serial.println("Test 0");

    // if (lookupTable != nullptr)
    // {
    //     // Delete the existing lookupTable

    //     Serial.print("TEST!!!");
    //     delete[] lookupTable;
    //     lookupTable = nullptr;
    // }



  result.angle_values.clear();
  result.velocity_values.clear();

Serial.println("Test 1");


   lookupTable = new AngleVelocityPairTable[ 2* num_accel_segments -1];

   Serial.println("Test 2");


   result.angle_values.resize(2* num_accel_segments+1 , 2* num_accel_segments+1 );
   result.velocity_values.resize(2* num_accel_segments+1 , 2* num_accel_segments+1 );

   Serial.println("Test 1");

  }else{

    num_values = 25;

    lookupTable = new AngleVelocityPairTable[num_values-1];


   result.angle_values.resize(num_values+1 , num_values +1);
   result.velocity_values.resize(num_values +1 , num_values +1);


  }

  float current_time = 0;

  float step_size_ad = acceleration_time / (num_accel_segments); // Calculate the step size between values

  float step_size_const = constVelocity_time / (num_const_segments);

  for (int i = 1; i <= num_values; ++i)
  {
    if (i <= num_accel_segments)
    {
      current_time = i * step_size_ad;

      if (i == 1)
      {

        // result.angle_values[i] = calculateDesiredPositionAtT(current_time, 0, step_size_ad);

        result.angle_values[i] = calculateDesiredPositionAtT(current_time, 0, step_size_ad);

      }
      else
      {
        result.angle_values[i] = result.angle_values[i - 1] + calculateDesiredPositionAtT(current_time, result.velocity_values[i - 1], step_size_ad);
       
        if (i == num_accel_segments)
        {
          constAngle = angleToGo - 2 * result.angle_values[i];
         
          step_size_const = (constAngle / commanded_velocity) / num_const_segments;

          if(constVelocity_time ==0){

              constVelocity_time = 0;
          }else{
            constVelocity_time = 0;
            constVelocity_time = constAngle / commanded_velocity;

          }

        }
      }

      result.velocity_values[i] = calculatedCommandVelocity(current_time, commanded_velocity);

        lookupTable[i-1].angle = result.angle_values[i];
        lookupTable[i-1].velocity = result.velocity_values[i];


      Serial.print(" ");
      Serial.print(current_time);
      Serial.print(" ");
      Serial.print(result.angle_values[i],4);
      Serial.print(" ");
      Serial.print(result.velocity_values[i],4);
      Serial.println();

      
    }

    if (i > num_accel_segments && i <= num_accel_segments + num_const_segments)
    {
 
      current_time = acceleration_time + (i - num_accel_segments) * step_size_const;

      result.angle_values[i] = result.angle_values[i - 1] + commanded_velocity * step_size_const;

      result.velocity_values[i] = calculatedCommandVelocity(current_time, commanded_velocity);

      lookupTable[i-1].angle = result.angle_values[i];
      lookupTable[i-1].velocity = result.velocity_values[i];

      Serial.print(" ");
      Serial.print(current_time);
      Serial.print(" ");
      Serial.print(result.angle_values[i],4);
      Serial.print(" ");
      Serial.print(result.velocity_values[i],4);
      Serial.println();
    }

    if (i > num_accel_segments + num_const_segments && i <= num_values)
    {

      current_time = acceleration_time + constVelocity_time + (i - num_accel_segments - num_const_segments) * step_size_ad;

      // Serial.print("acceleration_time");
      // Serial.print(acceleration_time);
      // Serial.print(" constVelocity_time: ");
      // Serial.print(constVelocity_time);
      // Serial.println();


      int rel_i = i - num_accel_segments - num_const_segments;

      result.angle_values[i] = result.angle_values[i - 1] + (result.angle_values[num_accel_segments + 1 - rel_i] - result.angle_values[num_accel_segments - rel_i]);

      result.velocity_values[i] = calculatedCommandVelocity(current_time, commanded_velocity);

      

      lookupTable[i-1].angle = result.angle_values[i];
      lookupTable[i-1].velocity = result.velocity_values[i];

      

      Serial.print(" ");
      Serial.print(current_time);
      Serial.print(" ");
      Serial.print(result.angle_values[i],4);
      Serial.print(" ");
      Serial.print(result.velocity_values[i],4);
      Serial.println();


    }
  }

  float end_time = micros();

 

  unsigned long duration = end_time - start_time;

  // Serial.println(duration);

   

  // return result;
}

SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE1);

float t = 0;

float sigmoid(float x, float c, float c2)
{
  return 1 / (1 + exp(-c * (x - c2)));
}

float calculateDesiredPositionAtT(float &current_time, float prevVel, float &step_size_ad)
{

  float currentAngle = step_size_ad * (prevVel + sigmoid(current_time, c, c2)) / 2;

  return currentAngle;
};

float calculatedCommandVelocity(float &t, float &comanded_velocity)
{

  float v_cmd = comanded_velocity;

  if (t <= acceleration_time)
  {
    v_cmd = comanded_velocity * sigmoid(t, c, c2);
  }
  if (t > acceleration_time && t <= constVelocity_time + acceleration_time)
  {
    v_cmd = comanded_velocity;
  }
  if (t > (constVelocity_time + acceleration_time))
  {
    v_cmd = comanded_velocity - comanded_velocity * sigmoid(t - acceleration_time - constVelocity_time, c, c2);
  }

  // t += 0.01; // time_interval, assuming a time interval of 50 ms

  return v_cmd;
}

void IRAM_ATTR MCPWM_ISR(void *)
{

  MCPWM0.int_clr.timer0_tez_int_clr = 1; // clear interrupt flag

  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 20);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, 20);

  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, 20);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, 20);

  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_A, 20);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_B, 20);
}










void setup()
{
  Serial.begin(115200);
  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH); // Deselect AS5048A initially

  pinMode(EN_GATE, OUTPUT);
  digitalWrite(EN_GATE, HIGH);

  setCpuFrequencyMhz(240);

  mcpwm_config_t pwm_config_A;
  pwm_config_A.frequency = freqCarr * 2;             // Set frequency in Hz
  pwm_config_A.counter_mode = MCPWM_UP_DOWN_COUNTER; // phase freq correct
  pwm_config_A.duty_mode = MCPWM_DUTY_MODE_0;        // active high PWM
  pwm_config_A.cmpr_a = 0.0;                         // duty cycle to 0%
  pwm_config_A.cmpr_b = 0.0;

  // Register ISR

  // mcpwm_isr_register(MCPWM_UNIT_0, MCPWM_ISR, NULL, ESP_INTR_FLAG_IRAM, NULL);

  pinMode(LO1, OUTPUT);
  pinMode(HO1, OUTPUT);

  pinMode(LO2, OUTPUT);
  pinMode(HO2, OUTPUT);

  pinMode(LO3, OUTPUT);
  pinMode(HO3, OUTPUT);

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config_A); // init PWM
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, LO1);            // Use GPIO 12 for MCPWM0A
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, HO1);            // Use GPIO 13 for MCPWM0B

  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, MCPWM_DUTY_MODE_1);
  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 5, 5);

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config_A); // init PWM
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, LO2);            // Use GPIO 12 for MCPWM0A
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, HO2);            // Use GPIO 13 for MCPWM0B

  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, MCPWM_DUTY_MODE_1);
  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 5, 5);

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config_A); // init PWM
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, LO3);            // Use GPIO 12 for MCPWM0A
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, HO3);            // Use GPIO 13 for MCPWM0B

  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_B, MCPWM_DUTY_MODE_1);
  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 5, 5);

  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1);
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_2);

  MCPWM0.int_ena.timer0_tez_int_ena = 1;


  // Lookup table initialization
  lookupTable = new AngleVelocityPairTable[num_values - 1];



}

float targetAngle = 0;
float targetVelocity = 0;


float interpolateLinear(float x, float x0, float x1, float y0, float y1) {
    return y0 + (y1 - y0) * ((x - x0) / (x1 - x0));
}



int findClosestAngleIndex(float angle) {
    int low = 0;
    int high = num_values +1;

    while (low <= high) {
        int mid = low + (high - low) / 2;
        
        // Check if angle is exactly at mid
        if (lookupTable[mid].angle == angle) {
            return mid;
        }
        // If angle is greater than mid, search in the right half
        else if (lookupTable[mid].angle < angle) {
            low = mid + 1;
        }
        // If angle is smaller than mid, search in the left half
        else {
            high = mid - 1;
        }
    }

    // Return the index of the closest angle
    if (high < 0) {
        return low;
    } else if (low >= num_values) {
        return high;
    } else {
        return (lookupTable[low].angle - angle) < (angle - lookupTable[high].angle) ? low : high;
    }
}

float getInterpolatedVelocity(float angle) {
    // Find the closest angle index using binary search
    int index = findClosestAngleIndex(angle);

    // Perform linear interpolation
    return interpolateLinear(angle, lookupTable[index - 1].angle, lookupTable[index].angle, lookupTable[index - 1].velocity, lookupTable[index].velocity);
}


void sendLookupTableToSerial() {

  int numRows = sizeof(lookupTable->angle) / sizeof(lookupTable[0]);

  Serial.print("numberOfrows");

  Serial.println(numRows);

    for (int i = 0; i < numRows; i++) {


        // Send angle and velocity for each row
        Serial.print("Angle: ");
        Serial.print(lookupTable[i].angle,5);
        Serial.print(" Velocity: ");
        Serial.println(lookupTable[i].velocity,5);
        //delay(100); // Delay between sending each pair of values (adjust as needed)
    }
}



void loop()
{

  // if (constVelocity_time < 0)
  // {

  //   max_velocity = 2 * (acceleration_time) / angleToGo;

  //   constVelocity_time = 0;
  // }


   if (Serial.available() > 0) {
    // Read the incoming byte
    char incomingByte = Serial.read();

    // Check if the received byte is the start of your command
    if (incomingByte == 'S') {
      // Read the rest of the command
      String command = Serial.readStringUntil('\n');


      Serial.println(command);


      // Process the command
      
        // Extract target angle and target velocity from the command
        int commaIndex = command.indexOf(',');
        if (commaIndex != -1) {
          String angleStr = command.substring(0, commaIndex);
          String velocityStr = command.substring(commaIndex + 2); // Skip comma and space

          // Convert strings to integers
          targetAngle = angleStr.toFloat();
          targetVelocity = velocityStr.toFloat();




          Serial.print(targetAngle,5);
          Serial.print(", ");
          Serial.print(targetVelocity,5);
          Serial.println();

          // by calling this function, Lookup table is created. 

          generateAngleAndVelocityValues(targetAngle, targetVelocity);

          sendLookupTableToSerial(); 
          // Serial.println();
          // Serial.print("Interpolated at 0.011: ");
          // Serial.print(getInterpolatedVelocity(0.011));
          // Serial.println();



          
          // From this moment on, I have to enter servo loop, to drive the motor to the target angle, 
          // following required velocit profile



        }
      }
    }
  

}

