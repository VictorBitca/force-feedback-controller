#include <SimpleFOC.h>
#include <Wire.h>
#include "I2C_Extension.h"

const byte SLAVE_ADDRESS = 42;

// BLDC motor & driver instance, make sure you use the correct pole pair count.
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

float target_voltage = 0;
float force = 0;

void setup() {
  Wire.begin (); 
  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 6;
  driver.init();
  
  motor.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  motor.init();
  motor.initFOC();
  _delay(1000);
}

void loop() {
  motor.loopFOC();
  
  Wire.requestFrom(SLAVE_ADDRESS, sizeof force);
  I2C_readAnything(force);

  motor.move(force);
  int16_t angle = (sensor.getAngle() * 4068) / 71;

  Wire.beginTransmission (SLAVE_ADDRESS);
  I2C_writeAnything(angle);
  Wire.endTransmission ();
}

float convertRawAngleToDegrees(word newAngle) {
  /* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */
  float retVal = newAngle * 0.087;
  return retVal;
}
