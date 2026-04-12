#pragma once

#include <Arduino.h>

// Global debug toggle. Set in main.cpp, checked by the DBG macro.
// When false, all DBG() calls are silently skipped at runtime.
extern bool debugSerialEnabled;

// Printf-style debug macro.  Tag is a short prefix like "PPG", "IMU", etc.
// Usage: DBG("PPG", "FIFO level=%u", level);
//   Output: [PPG] FIFO level=42
#define DBG(tag, fmt, ...)                                          \
    do {                                                            \
        if (debugSerialEnabled) {                                   \
            Serial.printf("[" tag "] " fmt "\n", ##__VA_ARGS__);    \
        }                                                           \
    } while (0)
