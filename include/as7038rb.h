#pragma once

#include <Wire.h>
#include <stdint.h>

void ppgInit( TwoWire& wire );
void ppgStartSampling();
void ppgStopSampling();
void ppgCollectSamples( uint32_t durationMs );
bool ppgGetHeartRate( uint16_t* bpm );
bool ppgGetSpO2( uint8_t* spo2 );
bool ppgGetHrv( uint16_t* rmssd );
void ppgPowerDown();
void ppgPowerUp();

// Drains all available FIFO triplets and returns the most recent red+IR+green sample.
// Call while sampling is running to read live raw photodiode counts.
// Returns false if fewer than three FIFO entries are available.
bool ppgReadLatestFifoSample( uint16_t* red, uint16_t* ir, uint16_t* green, uint8_t* fifoLevel );
