#include <Wire.h>
#include "Ffbwheel.h"
#include "PID_v1.h"
#include <Wire.h>
#include "I2C_Extension.h"

#define BAUD_RATE 115200

int32_t totalForce = 0;
int32_t last_total_force = 0;
double Setpoint, Input, Output;
double Kp = 0.1 , Ki = 30 , Kd =  0;

volatile float mappedForce = 0;
volatile int16_t sensorAngle = 0;
int16_t sensorAngleOfset = 0;

Wheel_ wheel;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// I2C adress
const byte MY_ADDRESS = 42;

// Buttons
int triggerA = A0;
int triggerB = A2;
int bumperA = A1;
int bumperB = A3;

int buttonA = 5;
int buttonB = 4;
int buttonX = 7;
int buttonY = 6;

bool aPressed = false;
bool bPressed = false;
bool xPressed = false;
bool yPressed = false;

void setup() {
  // Setup internal pull up resitors
  pinMode(triggerA, INPUT_PULLUP);
  pinMode(triggerB, INPUT_PULLUP);
  pinMode(bumperA, INPUT_PULLUP);
  pinMode(bumperB, INPUT_PULLUP);
  
  pinMode(buttonA, INPUT_PULLUP);
  pinMode(buttonB, INPUT_PULLUP);
  pinMode(buttonX, INPUT_PULLUP);
  pinMode(buttonY, INPUT_PULLUP);

  // Setup I2C
  Wire.begin(MY_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  Serial.begin(BAUD_RATE);

  // Setup wheel
  wheel.begin();

  Input = wheel.encoder.currentPosition;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(0.01);
  myPID.SetOutputLimits(-50, 50);
}

void loop() {
  int16_t mappedAcceleratorValue = map(analogRead(triggerA), 100, 1023, -32767, 32767);
  wheel.yAxis(mappedAcceleratorValue);

  int16_t mappedBrakeValue = map(analogRead(triggerB), 100, 1023, -32767, 32767);
  wheel.zAxis(mappedBrakeValue);

  int16_t mappedClutchValue = map(analogRead(bumperB), 0, 1023, -32767, 32767);
  wheel.rxAxis(mappedClutchValue);

  int16_t val = map(sensorAngle - sensorAngleOfset, 360, -360, -32768, 32767);
  wheel.encoder.updatePosition(val);
  wheel.xAxis(val);

  readAndUpdateButtonStatesIfNeeded();

  wheel.RecvFfbReport();
  wheel.write();

  totalForce = wheel.ffbEngine.ForceCalculator(wheel.encoder);
  float forceMultiplier = -0.3;
  mappedForce = mapf(totalForce, -255.0, 255.0, -5.0, 5.0) * forceMultiplier;
}

// handle I2C bus recieve events
void receiveEvent(int howMany) {
  if (howMany >= (sizeof sensorAngle)) {
    I2C_readAnything(sensorAngle);
  }
}

// handle I2C bus request events
void requestEvent() {
  I2C_writeAnything(mappedForce);
}

// Check the button states 
void readAndUpdateButtonStatesIfNeeded() {
  bool aState = digitalRead(buttonA);
  bool bState = digitalRead(buttonB);
  bool xState = digitalRead(buttonX);
  bool yState = digitalRead(buttonY);

  if (aState == aPressed) {
    aPressed = !aState;
    if (aState) {
      wheel.release(1);
    } else {
      wheel.press(1);
    }
  }

  if (bState == bPressed) {
    bPressed = !bState;
    if (bState) {
      wheel.release(2);
    } else {
      wheel.press(2);
    }
  }

  if (xState == xPressed) {
    xPressed = !xState;
    if (xState) {
      wheel.release(3);
    } else {
      wheel.press(3);
    }
  }

  if (yState == yPressed) {
    sensorAngleOfset = sensorAngle;
    yPressed = !yState;
    if (yState) {
      wheel.release(4);
    } else {
      wheel.press(4);
    }
  }
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
