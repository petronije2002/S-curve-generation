

#include <Arduino.h>
#include <SPI.h>
#include "MagneticEncoder.h"

#include <cstring>

// ng previousPosition = 0;


// const int csPin = 5; 
// float degAngle_;//Angle in degrees
uint16_t command = 0b1111111111111111; //read command (0xFFF)

uint16_t rawData = 0; //bits from the encoder (16 bits, but top 2 has to be discarded)


// float readAngle() {


  
//     // this function will read magnetic encoder value, and return its value in degrees 

//     SPI.beginTransaction(SPISettings(3000000, MSBFIRST, SPI_MODE1));

  
  
//     digitalWrite(csPin, LOW);  // Select AS5048A


//   // Send the command to read angle (address 0x3FFF)
//     SPI.transfer16(0x3FFF);


//     SPI.transfer16(command);


//     digitalWrite(csPin, HIGH);

//     delay(10);

//     digitalWrite(csPin, LOW);

//     // Read the response (16-bit angle value)
//     uint16_t angle123 = SPI.transfer16(0x0000);

//     digitalWrite(csPin, HIGH);  // Deselect AS5048A
//     SPI.endTransaction();

//     angle123 = angle123 & 0b0011111111111111; //removing the top 2 bits (PAR and EF)

//   //  (float)angle / 16384.0 * 360.0; //16384 = 2^14, 360 = 360 degrees

//   return  (float)angle123 / 16384.0 * 360.0; //16384 = 2^14, 360 = 360 degrees;
// }




const int numSamples = 10; // Number of samples for moving average
float angleSamples[numSamples]; // Array to store angle samples
int sampleIndex = 0; // Index for storing the current sample
// float degAngle_;//Angle in degrees











float readAngle1(const int &csPin=5) {


  // SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  SPI.beginTransaction(spiSettings);

  digitalWrite(csPin, LOW);
  SPI.transfer16(0x3FFF);
  digitalWrite(csPin, HIGH);
  //  delay(10);
  digitalWrite(csPin, LOW);
  uint16_t angle_ = SPI.transfer16(0x3FFF);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();

  angle_ = angle_ & 0b0011111111111111; // Removing the top 2 bits (PAR and EF)

  float degAngle_ = (float) angle_ / 16384.0 * 360.0; 

  

  return degAngle_;
};




 void readAngleTask(void *parameter) {
  uint16_t angle;

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(csPin, LOW);
  SPI.transfer16(0x3FFF);
  digitalWrite(csPin, HIGH);
  delayMicroseconds(10); // Introduce a small delay if necessary
  digitalWrite(csPin, LOW);
  angle = SPI.transfer16(0x3FFF);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();

  angle = angle & 0b0011111111111111; // Removing the top 2 bits (PAR and EF)

  currentPosition = (float)angle / 16384.0 * 360.0;

  // Add proper error handling if needed

  // Introduce a delay or rate-limiting mechanism if necessary
  vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100 milliseconds

  // Do something with degAngle, e.g., update a global variable, set a flag, etc.

}



void parseSerialInput(String input) {


  if(input.length()==16){


    recevedCommand = input;

    Serial.printf("Recevied for parsing: ", recevedCommand);


    return ;

  };


  // Example input format: A20.4,V300
  int angleIndex = input.indexOf('A');
  int velocityIndex = input.indexOf('V');

  if (angleIndex != -1 && velocityIndex != -1) {
    // Extract the target angle value
    String angleValueString = input.substring(angleIndex + 1, velocityIndex);
    targetPosition = angleValueString.toFloat();

    // Extract the target velocity value
    String velocityValueString = input.substring(velocityIndex + 1);
    targetVelocity = velocityValueString.toFloat();
  }
}


void updateVelocity() {
  // Calculate velocity based on position change
  float deltaTime = 0.01;  // Adjust this value based on your control loop frequency
  float positionChange = currentPosition - previousPosition;
  currentVelocity = positionChange / deltaTime;

  // Update previous position
  previousPosition = currentPosition;
}






uint16_t readAngle2(const int csPin) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(csPin, LOW);
  SPI.transfer16(0x3FFF);
  digitalWrite(csPin, HIGH);
  delay(10);
  digitalWrite(csPin, LOW);

  uint16_t angle_ = SPI.transfer16(0x3FFF);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();

  angle_ = angle_ & 0b0011111111111111; // Removing the top 2 bits (PAR and EF)

  // if ((angle_ <= 20)|| angle_ >= 16350){

  //   angle_ = 20;
  // }
  

  return angle_;
}
