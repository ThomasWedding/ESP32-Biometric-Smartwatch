#include "battery.h"
#include "debug.h"
#include <Adafruit_MAX1704X.h>
#include <Wire.h>

static Adafruit_MAX17048 maxLipo;
static bool batInitialized = false;

// Initialises the MAX17048 fuel gauge over I2C. Returns false if not found.
bool batInit()
{
    if( !maxLipo.begin( &Wire ) )
    {
        DBG( "BAT", "MAX17048 not found on I2C bus" );
        return false;
    }

    DBG( "BAT", "MAX17048 found - chip ID: 0x%02X, version: %u",
        maxLipo.getChipID(), maxLipo.getICversion() );
    DBG( "BAT", "Initial: %.1fV, %.1f%%", maxLipo.cellVoltage(), maxLipo.cellPercent() );

    batInitialized = true;
    return true;
}

// Returns the battery percentage (0–100), or -1.0 if the sensor is unavailable.
float batGetPercent()
{
    if( !batInitialized ) return -1.0f;
    float pct = maxLipo.cellPercent();
    if( pct > 100.0f ) pct = 100.0f;
    if( pct < 0.0f )   pct = 0.0f;
    return pct;
}

// Returns the battery voltage in volts, or -1.0 if the sensor is unavailable.
float batGetVoltage()
{
    if( !batInitialized ) return -1.0f;
    return maxLipo.cellVoltage();
}

// Returns true if the battery charge rate is positive (i.e. currently charging).
bool batIsCharging()
{
    if( !batInitialized ) return false;
    return maxLipo.chargeRate() > 0.1f;
}
