#include <Wire.h>
#include "Ffbwheel.h"
#include "PID_v1.h"
#include <Wire.h>
#include "I2C_Extension.h"

#define BAUD_RATE 9600

struct ControllerInput {
  long buttonSum;            // Sum of button presses
  long leftHorizontalAxis;   // Left joystick horizontal axis
  long leftVerticalAxis;     // Left joystick vertical axis
  long rightHorizontalAxis;  // Right joystick horizontal axis
  long brakeAxis;            // Brake axis
  long throttleAxis;         // Throttle axis
  long rightVerticalAxis;    // Right joystick vertical axis
};
ControllerInput controllerInput;

int32_t totalForce = 0;
int32_t last_total_force = 0;
double Setpoint, Input, Output;
double Kp = 0.1, Ki = 30, Kd = 0;

volatile float mappedForce = 0;
volatile int16_t sensorAngle = 0;
int16_t sensorAngleOfset = 0;

Wheel_ wheel;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// I2C adress
const byte MY_ADDRESS = 42;

// Serial buffer
const byte numChars = 128;
char receivedChars[numChars];
boolean newData = false;

void setup() {
  // Setup I2C
  Wire.begin(MY_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  // Setup serial communication
  Serial.begin(BAUD_RATE);
  Serial1.begin(BAUD_RATE);

  // Setup wheel
  wheel.begin();

  Input = wheel.encoder.currentPosition;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(0.01);
  myPID.SetOutputLimits(-50, 50);
}

void loop() {
  // Start reading data from Serial and parse it into the controllerInput struct
  recvWithStartEndMarkers();
  parseDataToControllerInputStruct();
  sendControllerButtonEvents();

  int16_t mappedAcceleratorValue = map(controllerInput.throttleAxis, 0, 1023, -32767, 32767);
  wheel.ryAxis(mappedAcceleratorValue);

  int16_t mappedBrakeValue = map(controllerInput.brakeAxis, 0, 1023, -32767, 32767);
  wheel.rzAxis(mappedBrakeValue);

  int16_t mappedXValue = controllerInput.rightHorizontalAxis;
  wheel.xAxis(mappedXValue);

  int16_t mappedYValue = controllerInput.rightVerticalAxis * -1;
  wheel.yAxis(mappedYValue);

  // int16_t mappedClutchValue = map(analogRead(bumperB), 0, 1023, -32767, 32767);
  // wheel.xAxis(mappedClutchValue);

  // Read the sensor value from the driver board and set it to the wheel
  int16_t val = sensorAngle;//map(sensorAngle - sensorAngleOfset, -32767, 32768, -32767, 32767);
  Serial.print("\nRaw HW Sensor Angle: ");
  Serial.print(val);

  wheel.encoder.updatePosition(val);
  wheel.rxAxis(val);

  wheel.RecvFfbReport();
  wheel.write();

  // Compute the force feedback and store it for later when driver will ask for it
  totalForce = wheel.ffbEngine.ForceCalculator(wheel.encoder);
  float forceMultiplier = 0.3;
  mappedForce = mapf(totalForce, -255.0, 255.0, -5.0, 5.0) * forceMultiplier;
}

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial1.available() > 0 && newData == false) {
    rc = Serial1.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } else {
        receivedChars[ndx] = '\0';  // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void parseDataToControllerInputStruct() {
  if (newData == true) {
    newData = false;

    showParsedData();

    char* strtokIndx;  // this is used by strtok() as an index
    // Parse the button sum
    strtokIndx = strtok(receivedChars, ",");
    controllerInput.buttonSum = atoi(strtokIndx);
    // Parse the rest
    strtokIndx = strtok(NULL, ",");
    controllerInput.leftHorizontalAxis = atol(strtokIndx);
    strtokIndx = strtok(NULL, ",");
    controllerInput.leftVerticalAxis = atol(strtokIndx);
    strtokIndx = strtok(NULL, ",");
    controllerInput.rightHorizontalAxis = atol(strtokIndx);
    strtokIndx = strtok(NULL, ",");
    controllerInput.brakeAxis = atol(strtokIndx);
    strtokIndx = strtok(NULL, ",");
    controllerInput.throttleAxis = atol(strtokIndx);
    strtokIndx = strtok(NULL, ",");
    controllerInput.rightVerticalAxis = atol(strtokIndx);
  }
}

void showParsedData() {
  Serial.print("\nRaw HW Serial Input: ");
  Serial.println(receivedChars);

  Serial.println("Got struct from HW serial:");
  Serial.println(controllerInput.buttonSum);
  Serial.println(controllerInput.leftHorizontalAxis);
  Serial.println(controllerInput.leftVerticalAxis);
  Serial.println(controllerInput.brakeAxis);
  Serial.println(controllerInput.throttleAxis);
  Serial.println(controllerInput.rightHorizontalAxis);
  Serial.println(controllerInput.rightVerticalAxis);
}

// Global variable to track the previous state
int previousSUM = 0;

void sendControllerButtonEvents() {
  long SUM = controllerInput.buttonSum;
    if (SUM == 0) {
        wheel.releaseAll();
        previousSUM = 0;
        return;
    }

    // Arrays to hold button values and their corresponding translated values
    int buttonValues[] = {16, 32, 64, 128, 8, 4, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768};
    int translatedValues[] = {1, 2, 3, 4, 7, 8, 11, 12, 13, 14, 5, 6, 9, 10};

    // Size of the arrays
    int numButtons = sizeof(buttonValues) / sizeof(buttonValues[0]);

    // Handle button releases
    for (int i = 0; i < numButtons; i++) {
        if ((previousSUM & buttonValues[i]) && !(SUM & buttonValues[i])) {
            wheel.release(translatedValues[i]);
        }
    }

    // Handle button presses
    for (int i = 0; i < numButtons; i++) {
        if ((SUM & buttonValues[i]) && !(previousSUM & buttonValues[i])) {
            wheel.press(translatedValues[i]);
        }
    }

    // Update the previousSUM to the current state
    previousSUM = SUM;
}

// handle I2C bus recieve events from the driver board
void receiveEvent(int howMany) {
  Serial.print("\nreceiveEvent");
  Serial.println(howMany);
  if (howMany >= (sizeof sensorAngle)) {
    I2C_readAnything(sensorAngle);
  }
}

int32_t receivedData[7];  // Array to store received data
byte receivedBytes = 0;   // Counter to track received bytes

void readUARTControllerData() {
  // Check if data is available from the slave device
  if (Serial1.available()) {

    Serial.write(Serial1.read());
    // Receive data from the slave device
    // while (Serial1.available() && receivedBytes < sizeof(receivedData)) {
    //   ((byte*)receivedData)[receivedBytes] = Serial1.read();
    //   receivedBytes++;
    // }

    // // Once all data is received, print it to the Serial Monitor
    // if (receivedBytes >= sizeof(receivedData)) {
    //   Serial.println("\n\nRec data:");
    //   for (int i = 0; i < 7; i++) {
    //     Serial.print("Element ");
    //     Serial.print(i);
    //     Serial.print(": ");
    //     Serial.println(receivedData[i]);
    //   }
    //   receivedBytes = 0; // Reset the counter for the next set of data
    // }
  }
}

// handle I2C bus request events from the driver board
void requestEvent() {
  I2C_writeAnything(mappedForce);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
