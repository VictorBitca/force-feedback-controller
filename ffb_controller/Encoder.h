#ifndef ENCODER_h
#define ENCODER_h

#include <Arduino.h>
#include "WheelConfig.h"

class Encoder {
  public:
    Encoder(void);
    ~Encoder(void);
    uint32_t cPR;
    uint16_t maxAngle;
    int32_t maxValue;
    int32_t  minValue;    
    
    bool inverted;
    uint32_t lastEncoderTime;
    
    int32_t  currentPosition;
    int32_t  lastPosition;
    int32_t  correctPosition;    
    int32_t  currentVelocity;
    int32_t  lastVelocity;
    int32_t  maxVelocity;
    int32_t  currentAcceleration;
    int32_t  maxAcceleration;
    int32_t  positionChange;
    int32_t  maxPositionChange;
    void setConfig(WheelConfig wheelConfig);
    void initVariables(void);
    void updatePosition(int32_t position);


  private:
   
    bool resetPosition;
    volatile bool currentPinA;
    volatile bool lastPinA;
    volatile bool currentPinB;
    volatile bool lastPinB;
    volatile bool currentPinZ;
    volatile bool lastPinZ;
    int8_t parsePosition();

    volatile int8_t oldState;
};

#endif
