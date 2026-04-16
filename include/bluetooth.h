#pragma once

#include "dataBuffer.h"
#include <stdbool.h>
#include <stdint.h>

void     bleInit();
bool     bleIsConnected();
bool     bleIsTimeSynced();
uint64_t bleGetUnixMs();
void     bleSendReading( const BiometricReading& reading );
void     bleFlushBuffer();
void     bleProcess();

// Sends a time-sync request notification; Android should respond by writing
// an 8-byte little-endian uint64_t Unix timestamp to the time-write characteristic.
void bleRequestTimeSync();

// Populates the output parameters from the ESP32 RTC and returns true.
// Returns false if the RTC has never been synced via BLE; caller should use
// its own fallback clock values in that case.
bool bleGetCurrentTime( uint8_t* month, uint8_t* day, uint16_t* year,
                        uint8_t* hour, uint8_t* minute, uint8_t* second, bool* isPm );
