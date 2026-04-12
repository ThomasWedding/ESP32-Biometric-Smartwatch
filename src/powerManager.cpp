#include "powerManager.h"
#include "debug.h"
#include "esp_sleep.h"
#include <Arduino.h>

static const uint64_t PWR_MS_TO_US = 1000ULL;

// Performs any one-time power manager setup. Currently a no-op placeholder.
void pmInit()
{
    DBG( "PWR", "Power manager initialised" );
}

// Puts the MCU into light sleep for durationMs milliseconds, then returns.
void pmLightSleep( uint32_t durationMs )
{
    DBG( "PWR", "Entering light sleep for %lu ms", ( unsigned long )durationMs );
    esp_sleep_enable_timer_wakeup( ( uint64_t )durationMs * PWR_MS_TO_US );
    esp_light_sleep_start();
    DBG( "PWR", "Woke from light sleep (cause=%d)", ( int )esp_sleep_get_wakeup_cause() );
}

// Puts the MCU into deep sleep for durationMs milliseconds. Does not return.
void pmDeepSleep( uint32_t durationMs )
{
    DBG( "PWR", "Entering deep sleep for %lu ms (will not return)", ( unsigned long )durationMs );
    esp_sleep_enable_timer_wakeup( ( uint64_t )durationMs * PWR_MS_TO_US );
    esp_deep_sleep_start();
    // Execution does not continue here after deep sleep
}

// Drives the sensor power-rail enable pin HIGH to power sensors.
void pmEnableSensors()
{
    DBG( "PWR", "Enabling sensor power rail" );
    // TODO: drive a GPIO power-rail enable pin HIGH to power sensors,
    //       then re-initialise I2C peripherals after rail stabilises
}

// Drives the sensor power-rail enable pin LOW to cut sensor power.
void pmDisableSensors()
{
    DBG( "PWR", "Disabling sensor power rail" );
    // TODO: drive a GPIO power-rail enable pin LOW to cut sensor power
    //       when not sampling, saving ~1–2 mA quiescent current
}
