#pragma once

#include <stdint.h>
#include <stdbool.h>

bool  batInit();
float batGetPercent();
float batGetVoltage();
bool  batIsCharging();
