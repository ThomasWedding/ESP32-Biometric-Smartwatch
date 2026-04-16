#pragma once

#include <stdint.h>
#include <stdbool.h>

struct BiometricReading {
    uint64_t timestamp;      // Unix ms (if timestampIsUnix) or millis() (if not)
    uint16_t heartRate;      // BPM
    uint8_t  spo2;           // SpO2 percentage (0–100)
    uint16_t hrv;            // RMSSD in milliseconds
    uint32_t stepCount;      // cumulative step count
    bool     valid;          // true if reading contains valid sensor data
    bool     timestampIsUnix;// true if timestamp is Unix epoch ms; false if millis()
};

static const uint16_t DATA_BUFFER_CAPACITY = 128;

void     bufInit();
bool     bufPush( const BiometricReading& reading );
bool     bufPop( BiometricReading& reading );
bool     bufPeek( BiometricReading& reading );
uint16_t bufGetCount();
bool     bufIsEmpty();
bool     bufIsFull();
void     bufClear();
