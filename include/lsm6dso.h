#pragma once

#include <Wire.h>
#include <stdint.h>

void imuInit( uint8_t pedometerDebounceSteps );
bool imuGetStepCount( uint32_t* steps );
void imuResetStepCount();
bool imuGetAcceleration( float* ax, float* ay, float* az );
bool imuGetAngularRate( float* gx, float* gy, float* gz );
bool imuEnableTiltDetection();
bool imuCheckTiltEvent();
void imuDisableGyro();
void imuEnableGyro();
// bool imuEnableFsmWristTilt(); // FSM lift-to-wake — see lsm6dso.cpp for implementation
