#include "powerManager.h"
#include "as7038rb.h"
#include "lsm6dso.h"
#include "debug.h"
#include "esp_sleep.h"
#include <Arduino.h>
#include <WiFi.h>

static const uint64_t PWR_MS_TO_US = 1000ULL;

// GPIO pins that are unused by any subsystem; pulled down to prevent floating-pin leakage.
static const int PWR_UNUSED_PINS[]    = { 1, 2, 5 };
static const int PWR_UNUSED_PIN_COUNT = 3;

// Disables WiFi, reduces CPU to 80 MHz, and pulls unused GPIO pins down to
// prevent leakage current. Must be called once at startup before sensor init.
void pmInit()
{
    WiFi.mode( WIFI_OFF );
    setCpuFrequencyMhz( 80 );

    for( int i = 0; i < PWR_UNUSED_PIN_COUNT; i++ )
    {
        pinMode( PWR_UNUSED_PINS[ i ], INPUT_PULLDOWN );
    }

    DBG( "PWR", "Power manager initialised (CPU=80MHz, WiFi=OFF, unused pins pulled down)" );
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

// Re-enables the PPG sensor (LDO+OSC on, registers reloaded) and the IMU gyroscope.
// Must be called before ppgStartSampling() at the start of each sampling window.
void pmEnableSensors()
{
    DBG( "PWR", "Enabling sensors" );
    imuEnableGyro();
    ppgPowerUp();
}

// Powers down the PPG sensor (LDO+OSC off, ~0.5 µA) and disables the IMU gyroscope.
// The IMU accelerometer stays on to keep the pedometer and tilt detection running.
void pmDisableSensors()
{
    DBG( "PWR", "Disabling sensors" );
    ppgPowerDown();
    imuDisableGyro();
}
