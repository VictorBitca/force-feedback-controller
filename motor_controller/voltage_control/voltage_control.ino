#include <SimpleFOC.h>
#include <Wire.h>
#include "I2C_Extension.h"

#define BAUD_RATE 9600
const byte SLAVE_ADDRESS = 42;

// BLDC motor & driver instance, make sure you use the correct pole pair count.
BLDCMotor motor = BLDCMotor(7);

BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

float force = 0;

void setup() {
  // Setup serial communication
  // Serial.begin(BAUD_RATE);

  Wire.begin (); 
  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 8;
  driver.voltage_limit = 3;
  driver.init();
  
  motor.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage; //!< parameter determining the torque control type

  motor.init();
  motor.initFOC();
  _delay(1000);
}

void loop() {
  motor.loopFOC();
  
  Wire.requestFrom(SLAVE_ADDRESS, sizeof force);
  I2C_readAnything(force);

  motor.move(force);
  int16_t newangle = mapRotationToInt16(sensor.getAngle()) * -1;
  // Serial.print("\n Angle: ");
  // Serial.print(newangle);

  Wire.beginTransmission (SLAVE_ADDRESS);
  I2C_writeAnything(newangle);
  Wire.endTransmission ();
}

// Function to cap and map radians to int16_t range
int16_t mapRotationToInt16(float radians) {
  // Define the maximum and minimum bounds in radians
  float maxRadians = 2 * PI;
  float minRadians = -2 * PI;

  // Cap the radians value within the range of -2*PI to 2*PI
  if (radians >= maxRadians) {
    radians = maxRadians;
  } else if (radians <= minRadians) {
    radians = minRadians;
  }

  // Map the capped radians value to the range of int16_t
  // int16_t ranges from -32768 to 32767
  return mapFloat(radians, minRadians, maxRadians, INT16_MIN + 1, INT16_MAX - 1);
}

// Function to map float values
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float convertRawAngleToDegrees(word newAngle) {
  /* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */
  float retVal = newAngle * 0.087;
  return retVal;
}
