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

// Drains all available FIFO entries and returns the most recent sample.
// In single-LED red mode, all three output pointers receive the same red OFE1 value.
// Returns false if no FIFO entries are available.
bool ppgReadLatestFifoSample( uint16_t* red, uint16_t* ir, uint16_t* green, uint8_t* fifoLevel );
