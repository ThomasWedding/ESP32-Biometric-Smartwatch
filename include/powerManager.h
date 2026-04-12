#pragma once

#include <stdint.h>

void pmInit();
void pmLightSleep( uint32_t durationMs );
void pmDeepSleep( uint32_t durationMs );
void pmEnableSensors();
void pmDisableSensors();
