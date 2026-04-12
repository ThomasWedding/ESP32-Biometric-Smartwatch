#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include "dataBuffer.h"
#include "as7038rb.h"
#include "lsm6dso.h"
#include "display.h"
#include "bluetooth.h"
#include "powerManager.h"
#include "battery.h"
#include "debug.h"

// Turn serial debug output on/off
bool debugSerialEnabled = true;

// Testing modes
//   0 - normal operation (all sensors, BLE, power management active)
//   1 - dummy data: sensor init skipped; random drifting values
//   2 - PPG sensor test: only AS7038RB is initialized
//   3 - IMU sensor test: only LSM6DSO is initialized
//   4 - real PPG + dummy IMU
//   5 - real IMU + dummy PPG
#define TESTING_MODE 5

static const uint32_t SAMPLE_WINDOW_MS   = 5000; // duration of each PPG acquisition
static const uint32_t SAMPLE_INTERVAL_MS = 5000; // idle period between acquisitions

// Fallback date and time displayed if the watch has never received a BLE time sync.
// Once Android sends a timestamp, bleGetCurrentTime() is used instead and these values
// are no longer shown.
static const uint8_t  CLOCK_MONTH  = 4;
static const uint8_t  CLOCK_DAY    = 10;
static const uint16_t CLOCK_YEAR   = 2026;
static const uint8_t  CLOCK_HOUR   = 12;
static const uint8_t  CLOCK_MINUTE = 0;
static const uint8_t  CLOCK_SECOND = 0;
static const bool     CLOCK_IS_PM  = true;

// How often to request a time sync from Android while connected.
// Reduce for testing; 30 minutes is a reasonable production value.
static const uint32_t TIME_SYNC_INTERVAL_MS = 60000; // 1 minute (testing default)

// Screen auto-off & lift-to-wake configuration
static const uint32_t SCREEN_TIMEOUT_MS       = 30000; // screen off after 30 s of inactivity
static const bool     AUTO_SCREEN_OFF_ENABLED = true;  // set false to keep screen always on

// Lift-to-wake acceleration thresholds (g).
// After a tilt interrupt, all axis conditions must be met to wake the screen.
// Should be tuned based on the physical orientation of the IMU on the wrist.
static const float WAKE_ACCEL_X_MIN = -1.0f;
static const float WAKE_ACCEL_X_MAX =  1.0f;
static const float WAKE_ACCEL_Y_MIN = -1.0f;
static const float WAKE_ACCEL_Y_MAX =  1.0f;
static const float WAKE_ACCEL_Z_MIN = -1.0f;
static const float WAKE_ACCEL_Z_MAX =  1.0f;

// Hardware pins
static const int BUTTON_D0_PIN = 0;  // GPIO0 - boot button, active LOW
static const int BUTTON_D1_PIN = 1;  // GPIO1 - active LOW
static const int BUTTON_D2_PIN = 2;  // GPIO2
static const int IMU_INT1_PIN  = 6;  // GPIO6 - LSM6DSO INT1 (tilt detection)

// Interrupt flags (set by ISRs, cleared by screenTask)
static volatile bool buttonWakeFlag = false;
static volatile bool tiltWakeFlag   = false;

// Whether the IMU is initialized in the current testing mode
static bool imuAvailable = false;

// ISR triggered on FALLING edge of button D0 (active LOW).
static void IRAM_ATTR buttonD0Isr()
{
    buttonWakeFlag = true;
}

// ISR triggered on RISING edge of IMU INT1 (tilt detection output).
static void IRAM_ATTR imuInt1Isr()
{
    tiltWakeFlag = true;
}

#if TESTING_MODE == 1 || TESTING_MODE == 4 || TESTING_MODE == 5
// Dummy sensor data. Drifts randomly each cycle to simulate live readings
static uint16_t dummyBpm   = 72;
static uint8_t  dummySpo2  = 98;
static uint16_t dummyHrv   = 45;
static uint32_t dummySteps = 0;

// Nudges each dummy metric by a small random amount, clamped to realistic ranges.
static void mainUpdateDummyValues()
{
    dummyBpm   = constrain( ( int )dummyBpm   + random( -3, 4 ),  40,  200 );
    dummySpo2  = constrain( ( int )dummySpo2  + random( -1, 2 ),  90,  100 );
    dummyHrv   = constrain( ( int )dummyHrv   + random( -5, 6 ),  10,  120 );
    dummySteps += random( 0, 8 );
}
#endif // TESTING_MODE == 1 || 4 || 5

#if TESTING_MODE == 0 || TESTING_MODE == 1 || TESTING_MODE == 4 || TESTING_MODE == 5
// Acquires biometric data each cycle, updates the display, and queues or sends readings.
// Runs on Core 0.
static void sensorTask( void* pvParameters )
{
    for( ;; )
    {
        uint16_t bpm   = 0;
        uint8_t  spo2  = 0;
        uint16_t hrv   = 0;
        uint32_t steps = 0;

#if TESTING_MODE == 1
        // All dummy values. Simulates the sample window, then uses dummy values
        vTaskDelay( pdMS_TO_TICKS( SAMPLE_WINDOW_MS ) );
        mainUpdateDummyValues();
        bpm   = dummyBpm;
        spo2  = dummySpo2;
        hrv   = dummyHrv;
        steps = dummySteps;
        DBG( "MAIN", "Dummy values: BPM=%u SpO2=%u HRV=%u Steps=%lu", bpm, spo2, hrv, ( unsigned long )steps );

#elif TESTING_MODE == 4
        // Real PPG + dummy IMU
        DBG( "MAIN", "Starting PPG sampling for %lu ms", ( unsigned long )SAMPLE_WINDOW_MS );
        ppgStartSampling();
        vTaskDelay( pdMS_TO_TICKS( SAMPLE_WINDOW_MS ) );
        ppgStopSampling();
        DBG( "MAIN", "PPG sampling complete" );

        ppgGetHeartRate( &bpm );
        ppgGetSpO2( &spo2 );
        ppgGetHrv( &hrv );

        mainUpdateDummyValues();
        steps = dummySteps;
        DBG( "MAIN", "Readings (real PPG, dummy IMU): BPM=%u SpO2=%u HRV=%u Steps=%lu", bpm, spo2, hrv, ( unsigned long )steps );

#elif TESTING_MODE == 5
        // Real IMU + dummy PPG
        vTaskDelay( pdMS_TO_TICKS( SAMPLE_WINDOW_MS ) );

        imuGetStepCount( &steps );

        mainUpdateDummyValues();
        bpm  = dummyBpm;
        spo2 = dummySpo2;
        hrv  = dummyHrv;
        DBG( "MAIN", "Readings (dummy PPG, real IMU): BPM=%u SpO2=%u HRV=%u Steps=%lu",
            bpm, spo2, hrv, ( unsigned long )steps );

#else
        // Mode 0: normal operation, all real sensors
        DBG( "MAIN", "Enabling sensors" );
        pmEnableSensors();

        // Collect PPG samples for SAMPLE_WINDOW_MS; algorithms run internally
        DBG( "MAIN", "Starting PPG sampling for %lu ms", ( unsigned long )SAMPLE_WINDOW_MS );
        ppgStartSampling();
        vTaskDelay( pdMS_TO_TICKS( SAMPLE_WINDOW_MS ) );
        ppgStopSampling();
        DBG( "MAIN", "PPG sampling complete" );

        ppgGetHeartRate( &bpm );
        ppgGetSpO2( &spo2 );
        ppgGetHrv( &hrv );
        imuGetStepCount( &steps );
        DBG( "MAIN", "Readings: BPM=%u SpO2=%u HRV=%u Steps=%lu",
            bpm, spo2, hrv, ( unsigned long )steps );
#endif

        // Build reading struct
        BiometricReading reading;
        reading.timestamp = millis();
        reading.heartRate = bpm;
        reading.spo2      = spo2;
        reading.hrv       = hrv;
        reading.stepCount = steps;
        reading.valid     = ( bpm > 0 );

        // Resolve display time: use RTC if synced via BLE, otherwise use fallback constants.
        uint8_t  clkMonth  = CLOCK_MONTH,  clkDay    = CLOCK_DAY;
        uint16_t clkYear   = CLOCK_YEAR;
        uint8_t  clkHour   = CLOCK_HOUR,   clkMinute = CLOCK_MINUTE;
        uint8_t  clkSecond = CLOCK_SECOND;
        bool     clkIsPm   = CLOCK_IS_PM;
        bleGetCurrentTime( &clkMonth, &clkDay, &clkYear, &clkHour, &clkMinute, &clkSecond, &clkIsPm );

        // Update display (-1 if battery unavailable, 100% if plugged in with no battery)
        int8_t batPct   = ( int8_t )batGetPercent();
        bool   charging = batIsCharging();
        dispUpdateMetrics( bpm, spo2, hrv, steps, bleIsConnected(), batPct, charging,
                           clkMonth, clkDay, clkYear,
                           clkHour, clkMinute, clkSecond, clkIsPm );

        // Only transmit or buffer valid readings
        if( reading.valid )
        {
            if( bleIsConnected() )
            {
                DBG( "MAIN", "Sending reading via BLE (t=%lu)", ( unsigned long )reading.timestamp );
                bleSendReading( reading );
            } else 
            {
                DBG( "MAIN", "BLE disconnected, buffering reading (buf=%u/%u)", bufGetCount(), DATA_BUFFER_CAPACITY );
                bufPush( reading );
            }
        } else {
            DBG( "MAIN", "Invalid reading (BPM=0), dropping" );
        }

#if TESTING_MODE == 1 || TESTING_MODE == 4 || TESTING_MODE == 5
        vTaskDelay( pdMS_TO_TICKS( SAMPLE_INTERVAL_MS ) );
#else
        DBG( "MAIN", "Disabling sensors, entering light sleep for %lu ms",
            ( unsigned long )SAMPLE_INTERVAL_MS );
        pmDisableSensors();

        // Light-sleep between acquisitions to conserve power
        pmLightSleep( SAMPLE_INTERVAL_MS );
        DBG( "MAIN", "Woke from light sleep" );
#endif
    }
}

// Flushes buffered readings and manages periodic time sync requests. Runs on Core 1.
// Sends an initial time-sync request ~1 s after connecting (gives Android time to
// subscribe to the notify characteristic), then re-requests every TIME_SYNC_INTERVAL_MS.
static void bleTask( void* pvParameters )
{
    bool     prevConnected    = false;
    uint32_t connectedSinceMs = 0;
    bool     initialSyncSent  = false;
    uint32_t lastTimeSyncMs   = 0;

    for( ;; )
    {
        bleProcess();

        bool connected = bleIsConnected();

        if( connected && !prevConnected )
        {
            // Fresh connection: record timestamp and arm the initial sync
            connectedSinceMs = millis();
            initialSyncSent  = false;
        }

        if( connected )
        {
            // Initial request fires ~1 s after connect so Android can subscribe first
            if( !initialSyncSent && ( millis() - connectedSinceMs >= 1000 ) )
            {
                bleRequestTimeSync();
                lastTimeSyncMs  = millis();
                initialSyncSent = true;
            }

            // Periodic refresh while connected
            if( initialSyncSent && ( millis() - lastTimeSyncMs >= TIME_SYNC_INTERVAL_MS ) )
            {
                bleRequestTimeSync();
                lastTimeSyncMs = millis();
            }
        }

        prevConnected = connected;
        vTaskDelay( pdMS_TO_TICKS( 100 ) );
    }
}
#endif // TESTING_MODE == 0 || 1 || 4 || 5

#if TESTING_MODE == 2
// Starts the AS7038RB sequencer and polls the FIFO every 100 ms, printing raw
// red and IR photodiode counts to serial and the display (TESTING_MODE 2 only).
static void ppgTestTask( void* pvParameters )
{
    ppgStartSampling();

    for( ;; )
    {
        uint16_t red       = 0;
        uint16_t ir        = 0;
        uint16_t green     = 0;
        uint8_t  fifoLevel = 0;

        if( ppgReadLatestFifoSample( &red, &ir, &green, &fifoLevel ) )
        {
            Serial.printf( "[PPG] RED=%5u  IR=%5u  GREEN=%5u  FIFO=%3u\n", red, ir, green, fifoLevel );
            dispShowPpgRaw( red, ir, green, fifoLevel );
        } else {
            Serial.printf( "[PPG] FIFO empty (level=%u) - waiting\n", fifoLevel );
        }

        vTaskDelay( pdMS_TO_TICKS( 100 ) );
    }
}
#endif // TESTING_MODE == 2

#if TESTING_MODE == 3
// Polls accelerometer, gyroscope, and step counter every 250 ms, printing raw values
// to serial and the display (TESTING_MODE 3 only).
static void imuTestTask( void* pvParameters )
{
    for( ;; )
    {
        float    ax = 0.0f, ay = 0.0f, az = 0.0f;
        float    gx = 0.0f, gy = 0.0f, gz = 0.0f;
        uint32_t steps = 0;

        bool accelOk = imuGetAcceleration( &ax, &ay, &az );
        bool gyroOk  = imuGetAngularRate( &gx, &gy, &gz );
        bool stepsOk = imuGetStepCount( &steps );

        if( accelOk && gyroOk && stepsOk )
        {
            Serial.printf(
                "[IMU] Accel(g): X=%+.3f Y=%+.3f Z=%+.3f  "
                "Gyro(dps): X=%+.3f Y=%+.3f Z=%+.3f  "
                "Steps=%lu\n",
                ax, ay, az, gx, gy, gz, ( unsigned long )steps
            );
            dispShowImuRaw( ax, ay, az, gx, gy, gz, steps );
        } else {
            Serial.printf(
                "[IMU] Read error - accel=%d gyro=%d steps=%d\n",
                accelOk, gyroOk, stepsOk
            );
        }

        vTaskDelay( pdMS_TO_TICKS( 250 ) );
    }
}
#endif // TESTING_MODE == 3

// Handles screen auto-off timeout and wake via button D0 or lift-to-wake. Runs on Core 1.
static void screenTask( void* pvParameters )
{
    bool     screenOn         = true;
    uint32_t lastActivityTime = millis();

    for( ;; )
    {
        // Button wake
        if( buttonWakeFlag )
        {
            buttonWakeFlag = false;
            if( !screenOn )
            {
                DBG( "MAIN", "Screen wake: button D0 pressed" );
                dispWake();
                screenOn = true;
            }
            lastActivityTime = millis();
        }

        // Lift-to-wake (only when IMU is available, routed to INT1).
        if( imuAvailable && tiltWakeFlag )
        {
            tiltWakeFlag = false;
            DBG( "MAIN", "Tilt triggered via GPIO interrupt" );

            float ax = 0.0f, ay = 0.0f, az = 0.0f;
            bool accelOk = imuGetAcceleration( &ax, &ay, &az );
            DBG( "MAIN", "Tilt accel: ok=%d ax=%.2f ay=%.2f az=%.2f",
                accelOk, ax, ay, az );

            if( accelOk &&
                ax >= WAKE_ACCEL_X_MIN && ax <= WAKE_ACCEL_X_MAX &&
                ay >= WAKE_ACCEL_Y_MIN && ay <= WAKE_ACCEL_Y_MAX &&
                az >= WAKE_ACCEL_Z_MIN && az <= WAKE_ACCEL_Z_MAX )
            {
                if( !screenOn )
                {
                    DBG( "MAIN", "Screen wake: lift-to-wake" );
                    dispWake();
                    screenOn = true;
                }
                lastActivityTime = millis();
            } else {
                DBG( "MAIN", "Tilt rejected: orientation outside thresholds" );
            }
        }

        // Auto screen off
        if( screenOn && AUTO_SCREEN_OFF_ENABLED &&
            ( millis() - lastActivityTime >= SCREEN_TIMEOUT_MS ) )
        {
            DBG( "MAIN", "Screen off: timeout (%lu ms)", ( unsigned long )SCREEN_TIMEOUT_MS );
            dispSleep();
            screenOn = false;
        }

        vTaskDelay( pdMS_TO_TICKS( 100 ) );
    }
}

// Initializes peripherals and creates FreeRTOS tasks.
void setup()
{
    Serial.begin( 115200 );
    // Wait up to 2s so startup log messages aren't dropped.
    // Continues immediately if no monitor is attached.
    {
        uint32_t deadline = millis() + 2000;
        while( !Serial && millis() < deadline );
    }

    DBG( "BOOT", "I2C bus init" );
    Wire.begin();

    WiFi.mode( WIFI_OFF );
    bufInit();
    dispInit();
    dispShowSplash();

    // Button D0 interrupt. Available in all modes for screen wake
    pinMode( BUTTON_D0_PIN, INPUT_PULLUP );
    attachInterrupt( digitalPinToInterrupt( BUTTON_D0_PIN ), buttonD0Isr, FALLING );
    DBG( "BOOT", "Button D0 (GPIO%d) interrupt attached", BUTTON_D0_PIN );

#if TESTING_MODE == 0
    // Normal operation. Initialize all sensors, BLE, and power management
    DBG( "BOOT", "Mode 0: normal operation" );
    batInit();
    imuInit();
    imuEnableTiltDetection();
    imuAvailable = true;
    ppgInit();
    bleInit();
    pmInit();

    // IMU INT1 interrupt for lift-to-wake
    pinMode( IMU_INT1_PIN, INPUT );
    attachInterrupt( digitalPinToInterrupt( IMU_INT1_PIN ), imuInt1Isr, RISING );
    DBG( "BOOT", "IMU INT1 (GPIO%d) interrupt attached", IMU_INT1_PIN );

    DBG( "BOOT", "Creating sensorTask (core 0) and bleTask (core 1)" );
    xTaskCreatePinnedToCore( sensorTask, "sensorTask", 8192, nullptr, 1, nullptr, 0 );
    xTaskCreatePinnedToCore( bleTask,    "bleTask",     4096, nullptr, 1, nullptr, 1 );

#elif TESTING_MODE == 1
    // Dummy-data mode. No sensor or BLE initialization; dummy values drive the UI
    DBG( "BOOT", "Mode 1: dummy data (no sensor init)" );
    batInit();
    randomSeed( esp_random() );
    bleInit();
    pmInit();

    DBG( "BOOT", "Creating sensorTask (core 0) and bleTask (core 1)" );
    xTaskCreatePinnedToCore( sensorTask, "sensorTask", 8192, nullptr, 1, nullptr, 0 );
    xTaskCreatePinnedToCore( bleTask,    "bleTask",     4096, nullptr, 1, nullptr, 1 );

#elif TESTING_MODE == 2
    // Only AS7038RB initialized; raw FIFO counts shown on display + serial
    DBG( "BOOT", "Mode 2: PPG raw test (AS7038RB only)" );
    ppgInit( Wire );

    xTaskCreatePinnedToCore( ppgTestTask, "ppgTestTask", 4096, nullptr, 1, nullptr, 0 );

#elif TESTING_MODE == 3
    // Only LSM6DSO initialized; raw accel/gyro/steps shown on display + serial
    DBG( "BOOT", "Mode 3: IMU raw test (LSM6DSO only)" );
    imuInit();
    imuEnableTiltDetection();
    imuAvailable = true;

    // IMU INT1 interrupt for lift-to-wake
    pinMode( IMU_INT1_PIN, INPUT );
    attachInterrupt( digitalPinToInterrupt( IMU_INT1_PIN ), imuInt1Isr, RISING );
    DBG( "BOOT", "IMU INT1 (GPIO%d) interrupt attached", IMU_INT1_PIN );

    xTaskCreatePinnedToCore( imuTestTask, "imuTestTask", 4096, nullptr, 1, nullptr, 0 );

#elif TESTING_MODE == 4
    // Real PPG + dummy IMU. Step count uses dummy values
    DBG( "BOOT", "Mode 4: real PPG + dummy IMU" );
    batInit();
    randomSeed( esp_random() );
    ppgInit( Wire );
    bleInit();

    DBG( "BOOT", "Creating sensorTask (core 0) and bleTask (core 1)" );
    xTaskCreatePinnedToCore( sensorTask, "sensorTask", 8192, nullptr, 1, nullptr, 0 );
    xTaskCreatePinnedToCore( bleTask,    "bleTask",     4096, nullptr, 1, nullptr, 1 );

#elif TESTING_MODE == 5
    // Real IMU + dummy PPG. HR/SpO2/HRV use dummy values
    DBG( "BOOT", "Mode 5: real IMU + dummy PPG" );
    batInit();
    randomSeed( esp_random() );
    imuInit();
    imuEnableTiltDetection();
    imuAvailable = true;
    bleInit();

    // IMU INT1 interrupt for lift-to-wake
    pinMode( IMU_INT1_PIN, INPUT );
    attachInterrupt( digitalPinToInterrupt( IMU_INT1_PIN ), imuInt1Isr, RISING );
    DBG( "BOOT", "IMU INT1 (GPIO%d) interrupt attached", IMU_INT1_PIN );

    DBG( "BOOT", "Creating sensorTask (core 0) and bleTask (core 1)" );
    xTaskCreatePinnedToCore( sensorTask, "sensorTask", 8192, nullptr, 1, nullptr, 0 );
    xTaskCreatePinnedToCore( bleTask,    "bleTask",     4096, nullptr, 1, nullptr, 1 );
#endif

    // Screen management that runs in all modes
    xTaskCreatePinnedToCore( screenTask, "screenTask", 4096, nullptr, 1, nullptr, 1 );
    DBG( "BOOT", "Screen task created (timeout=%lums, auto-off=%s, IMU wake=%s)",
        ( unsigned long )SCREEN_TIMEOUT_MS,
        AUTO_SCREEN_OFF_ENABLED ? "on" : "off",
        imuAvailable ? "on" : "off" );

    DBG( "BOOT", "Setup complete, entering task scheduler" );
}

// All work is done in FreeRTOS tasks; Arduino loop is suspended.
void loop()
{
    vTaskDelay( portMAX_DELAY );
}
