#pragma once

#include <stdint.h>
#include <stdbool.h>

void dispInit();
void dispShowSplash();
void dispUpdateMetrics( uint16_t bpm, uint8_t spo2, uint16_t hrv, uint32_t steps,
                        bool bleConnected, int8_t batteryPercent, bool charging,
                        uint8_t month, uint8_t day, uint16_t year,
                        uint8_t hour, uint8_t minute, uint8_t second, bool isPm );
void dispShowStatus( const char* msg );
void dispSetBrightness( uint8_t level );
void dispSleep();
void dispWake();

// Raw sensor test screens (used by TESTING_MODE 2 and 3)
void dispShowPpgRaw( uint16_t red, uint16_t ir, uint16_t green, uint8_t fifoLevel );
void dispShowImuRaw( float ax, float ay, float az,
                     float gx, float gy, float gz,
                     uint32_t steps );
