#include "display.h"
#include "debug.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Arduino.h>

// Hardware pin definitions for Adafruit ESP32-S3 Feather ReverseTFT
static const uint8_t TFT_CS_PIN       = 42; // GPIO42 per board pinout (GPIO7 is TFT_I2C_POWER)
static const uint8_t TFT_DC_PIN       = 40; // GPIO40
static const uint8_t TFT_RST_PIN      = 41; // GPIO41
static const uint8_t TFT_BACKLITE_PIN = 45; // GPIO45

// Display resolution
static const uint16_t DISP_WIDTH  = 240;
static const uint16_t DISP_HEIGHT = 135;

// Colour palette
static const uint16_t DISP_COLOR_BG      = ST77XX_BLACK;
static const uint16_t DISP_COLOR_TITLE   = ST77XX_WHITE;
static const uint16_t DISP_COLOR_HR      = ST77XX_GREEN;
static const uint16_t DISP_COLOR_SPO2    = ST77XX_CYAN;
static const uint16_t DISP_COLOR_HRV     = ST77XX_YELLOW;
static const uint16_t DISP_COLOR_STEPS   = ST77XX_WHITE;
static const uint16_t DISP_COLOR_BLE_ON  = ST77XX_BLUE;
static const uint16_t DISP_COLOR_BLE_OFF = 0x4208; // dark grey

// ST7789 instance (uses hardware SPI)
static Adafruit_ST7789 tft( TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN );

// Tracks whether dispUpdateMetrics has drawn the static chrome (background + separator).
// Reset to false after any full-screen repaint so the next dispUpdateMetrics call
// re-initialises the background before performing partial updates.
static bool dispMetricsInitialized = false;

// Previous values for each display region. Sentinel values ensure every region
// draws on the first call. Updated after each successful draw.
static uint16_t dispPrevBpm      = 0xFFFF;
static uint8_t  dispPrevSpo2     = 0xFF;
static uint16_t dispPrevHrv      = 0xFFFF;
static uint32_t dispPrevSteps    = 0xFFFFFFFF;
static bool     dispPrevBle      = false;
static int8_t   dispPrevBatPct   = -2;
static bool     dispPrevCharging = false;
static uint8_t  dispPrevMonth    = 0;
static uint8_t  dispPrevDay      = 0;
static uint16_t dispPrevYear     = 0;
static uint8_t  dispPrevHour     = 0xFF;
static uint8_t  dispPrevMinute   = 0xFF;
static uint8_t  dispPrevSecond   = 0xFF;
static bool     dispPrevIsPm     = false;

// Clears a region and draws a numeric value with a smaller label below it.
static void dispDrawValueWithLabel( int16_t x, int16_t y,
                                    const char* label, uint32_t value,
                                    uint16_t valueColor, uint8_t valueSize,
                                    uint16_t labelColor, uint8_t labelSize )
{
    // Clear the region first (wide enough for 5-digit number + label)
    tft.fillRect( x, y, 120, 40, DISP_COLOR_BG );

    // Value
    tft.setTextColor( valueColor );
    tft.setTextSize( valueSize );
    tft.setCursor( x, y );
    tft.print( value );

    // Label below value
    tft.setTextColor( labelColor );
    tft.setTextSize( labelSize );
    tft.setCursor( x, y + valueSize * 8 + 2 );
    tft.print( label );
}

// Initialises the ST7789 TFT display and turns the backlight on.
void dispInit()
{
    DBG( "DISP", "Init: CS=%u DC=%u RST=%u BL=%u (%ux%u)",
        TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN, TFT_BACKLITE_PIN,
        DISP_WIDTH, DISP_HEIGHT );

    pinMode( TFT_BACKLITE_PIN, OUTPUT );
    digitalWrite( TFT_BACKLITE_PIN, HIGH );

    tft.init( DISP_HEIGHT, DISP_WIDTH );  // height, width for landscape
    tft.setRotation( 3 );
    tft.fillScreen( DISP_COLOR_BG );

    DBG( "DISP", "Init complete" );
}

// Renders a centred splash screen with the product name and an initialising message.
void dispShowSplash()
{
    // The splash overwrites the full screen, so the next dispUpdateMetrics call must
    // re-initialise the background before doing partial updates.
    dispMetricsInitialized = false;

    tft.fillScreen( DISP_COLOR_BG );

    // "BIOWATCH" centred
    tft.setTextColor( ST77XX_WHITE );
    tft.setTextSize( 3 );
    int16_t tx, ty;
    uint16_t tw, th;
    tft.getTextBounds( "BIOWATCH", 0, 0, &tx, &ty, &tw, &th );
    tft.setCursor( ( DISP_WIDTH - tw ) / 2, ( DISP_HEIGHT / 2 ) - th - 4 );
    tft.print( "BIOWATCH" );

    // "Initializing..." centred below
    tft.setTextSize( 1 );
    tft.setTextColor( ST77XX_YELLOW );
    tft.getTextBounds( "Initializing...", 0, 0, &tx, &ty, &tw, &th );
    tft.setCursor( ( DISP_WIDTH - tw ) / 2, ( DISP_HEIGHT / 2 ) + 8 );
    tft.print( "Initializing..." );
}

// Redraws only the display regions whose values have changed since the last call.
// On the first call after init or a full-screen repaint, fills the background and draws
// the static separator line, then draws all regions unconditionally.
void dispUpdateMetrics( uint16_t bpm, uint8_t spo2, uint16_t hrv, uint32_t steps,
                        bool bleConnected, int8_t batteryPercent, bool charging,
                        uint8_t month, uint8_t day, uint16_t year,
                        uint8_t hour, uint8_t minute, uint8_t second, bool isPm )
{
    // Layout constants
    const int16_t col1X  = 4;   // left-column start x
    const int16_t row1Y  = 22;  // top metric value y
    const int16_t row2Y  = 95;  // bottom metric value y
    const int16_t lblOff = 18;  // label y offset from value start (16 px text + 2 px gap)

    // First call after init or a full-screen repaint: fill background, draw separator,
    // and reset all prev values so every region below triggers its redraw.
    if( !dispMetricsInitialized )
    {
        tft.fillScreen( DISP_COLOR_BG );
        tft.drawFastHLine( 0, 16, DISP_WIDTH, 0x2945 );
        dispMetricsInitialized = true;
        dispPrevBpm      = 0xFFFF;
        dispPrevSpo2     = 0xFF;
        dispPrevHrv      = 0xFFFF;
        dispPrevSteps    = 0xFFFFFFFF;
        dispPrevBatPct   = -2;
        dispPrevCharging = !charging;
        dispPrevBle      = !bleConnected;
        dispPrevMonth    = 0;
        dispPrevDay      = 0;
        dispPrevYear     = 0;
        dispPrevHour     = 0xFF;
        dispPrevMinute   = 0xFF;
        dispPrevSecond   = 0xFF;
        dispPrevIsPm     = !isPm;
    }

    // Date (top-left). Only redraws on date change
    if( month != dispPrevMonth || day != dispPrevDay || year != dispPrevYear )
    {
        tft.fillRect( 4, 0, 66, 16, DISP_COLOR_BG );
        char dateStr[ 11 ];
        snprintf( dateStr, sizeof( dateStr ), "%02u/%02u/%04u", month, day, year );
        tft.setTextColor( DISP_COLOR_TITLE );
        tft.setTextSize( 1 );
        tft.setCursor( 4, 4 );
        tft.print( dateStr );
        dispPrevMonth = month;
        dispPrevDay   = day;
        dispPrevYear  = year;
    }

    // Battery indicator. Only redraws on percent or charging state change
    if( batteryPercent != dispPrevBatPct || charging != dispPrevCharging )
    {
        tft.fillRect( 70, 0, 154, 16, DISP_COLOR_BG );
        if( batteryPercent >= 0 )
        {
            uint16_t batColor;
            if( charging )                  batColor = ST77XX_CYAN;
            else if( batteryPercent <= 15 ) batColor = ST77XX_RED;
            else if( batteryPercent <= 30 ) batColor = ST77XX_YELLOW;
            else                            batColor = ST77XX_GREEN;

            char batStr[ 12 ];
            if( charging )
                snprintf( batStr, sizeof( batStr ), "CHG %d%%", batteryPercent );
            else
                snprintf( batStr, sizeof( batStr ), "%d%%", batteryPercent );

            int16_t  tx, ty;
            uint16_t tw, th;
            tft.setTextSize( 1 );
            tft.getTextBounds( batStr, 0, 0, &tx, &ty, &tw, &th );
            tft.setTextColor( batColor );
            tft.setCursor( DISP_WIDTH - 16 - tw, 4 );
            tft.print( batStr );
        }
        dispPrevBatPct   = batteryPercent;
        dispPrevCharging = charging;
    }

    // BLE dot (top-right). Only redraws on connection state change
    if( bleConnected != dispPrevBle )
    {
        uint16_t bleColor = bleConnected ? DISP_COLOR_BLE_ON : DISP_COLOR_BLE_OFF;
        tft.fillCircle( DISP_WIDTH - 8, 7, 4, bleColor );
        dispPrevBle = bleConnected;
    }

    // Heart Rate (top-left). Only redraws on BPM change
    if( bpm != dispPrevBpm )
    {
        tft.fillRect( col1X, row1Y, 72, 30, DISP_COLOR_BG );
        tft.setTextColor( DISP_COLOR_HR );
        tft.setTextSize( 2 );
        tft.setCursor( col1X, row1Y );
        tft.print( bpm );
        tft.setTextSize( 1 );
        tft.setCursor( col1X, row1Y + lblOff );
        tft.print( "BPM" );
        dispPrevBpm = bpm;
    }

    // SpO2 (top-right, right-aligned). Only redraws on SpO2 change
    if( spo2 != dispPrevSpo2 )
    {
        tft.fillRect( DISP_WIDTH - 72, row1Y, 72, 30, DISP_COLOR_BG );
        char spo2Str[ 4 ];
        snprintf( spo2Str, sizeof( spo2Str ), "%u", spo2 );
        tft.setTextColor( DISP_COLOR_SPO2 );
        tft.setTextSize( 2 );
        tft.setCursor( DISP_WIDTH - 4 - ( int16_t )( strlen( spo2Str ) * 12 ), row1Y );
        tft.print( spo2Str );
        tft.setTextSize( 1 );
        tft.setCursor( DISP_WIDTH - 4 - ( int16_t )( strlen( "SpO2 %" ) * 6 ), row1Y + lblOff );
        tft.print( "SpO2 %" );
        dispPrevSpo2 = spo2;
    }

    // HRV (bottom-left). Only redraws on HRV change
    if( hrv != dispPrevHrv )
    {
        tft.fillRect( col1X, row2Y, 72, 30, DISP_COLOR_BG );
        tft.setTextColor( DISP_COLOR_HRV );
        tft.setTextSize( 2 );
        tft.setCursor( col1X, row2Y );
        tft.print( hrv );
        tft.setTextSize( 1 );
        tft.setCursor( col1X, row2Y + lblOff );
        tft.print( "HRV ms" );
        dispPrevHrv = hrv;
    }

    // Steps (bottom-right, right-aligned). Only redraws on step count change
    if( steps != dispPrevSteps )
    {
        tft.fillRect( DISP_WIDTH - 116, row2Y, 116, 30, DISP_COLOR_BG );
        char stepsStr[ 11 ];
        snprintf( stepsStr, sizeof( stepsStr ), "%lu", ( unsigned long )steps );
        tft.setTextColor( DISP_COLOR_STEPS );
        tft.setTextSize( 2 );
        tft.setCursor( DISP_WIDTH - 4 - ( int16_t )( strlen( stepsStr ) * 12 ), row2Y );
        tft.print( stepsStr );
        tft.setTextSize( 1 );
        tft.setCursor( DISP_WIDTH - 4 - ( int16_t )( strlen( "STEPS" ) * 6 ), row2Y + lblOff );
        tft.print( "STEPS" );
        dispPrevSteps = steps;
    }

    // Time (center). Only redraws when any time component changes
    if( hour != dispPrevHour || minute != dispPrevMinute ||
        second != dispPrevSecond || isPm != dispPrevIsPm )
    {
        tft.fillRect( 60, 55, 120, 36, DISP_COLOR_BG );
        char timeStr[ 9 ];
        snprintf( timeStr, sizeof( timeStr ), "%02u:%02u:%02u", hour, minute, second );
        tft.setTextColor( DISP_COLOR_TITLE );
        tft.setTextSize( 2 );
        tft.setCursor( 72, 63 );
        tft.print( timeStr );
        tft.setTextSize( 1 );
        tft.setCursor( ( DISP_WIDTH - 12 ) / 2, 81 );
        tft.print( isPm ? "PM" : "AM" );
        dispPrevHour   = hour;
        dispPrevMinute = minute;
        dispPrevSecond = second;
        dispPrevIsPm   = isPm;
    }
}

// Clears the screen and displays a single centred status message.
void dispShowStatus( const char* msg )
{
    tft.fillScreen( DISP_COLOR_BG );
    tft.setTextColor( ST77XX_WHITE );
    tft.setTextSize( 1 );

    int16_t tx, ty;
    uint16_t tw, th;
    tft.getTextBounds( msg, 0, 0, &tx, &ty, &tw, &th );
    tft.setCursor( ( DISP_WIDTH - tw ) / 2, ( DISP_HEIGHT - th ) / 2 );
    tft.print( msg );
}

// Sets the backlight brightness via PWM (0 = off, 255 = full).
void dispSetBrightness( uint8_t level )
{
    analogWrite( TFT_BACKLITE_PIN, level );
}

// Renders raw PPG FIFO data (red, IR, green counts and FIFO level) on screen.
void dispShowPpgRaw( uint16_t red, uint16_t ir, uint16_t green, uint8_t fifoLevel )
{
    tft.fillScreen( DISP_COLOR_BG );

    // Title
    tft.setTextColor( ST77XX_MAGENTA );
    tft.setTextSize( 2 );
    tft.setCursor( 4, 4 );
    tft.print( "PPG RAW TEST" );

    // Red channel
    tft.setTextColor( ST77XX_RED );
    tft.setTextSize( 2 );
    tft.setCursor( 4, 26 );
    tft.print( "RED: " );
    tft.print( red );

    // IR channel
    tft.setTextColor( ST77XX_CYAN );
    tft.setTextSize( 2 );
    tft.setCursor( 4, 52 );
    tft.print( "IR:  " );
    tft.print( ir );

    // Green channel
    tft.setTextColor( ST77XX_GREEN );
    tft.setTextSize( 2 );
    tft.setCursor( 4, 78 );
    tft.print( "GRN: " );
    tft.print( green );

    // FIFO level
    tft.setTextColor( ST77XX_YELLOW );
    tft.setTextSize( 1 );
    tft.setCursor( 4, 110 );
    tft.print( "FIFO level: " );
    tft.print( fifoLevel );
    tft.print( " / 127" );
}

// Renders raw IMU accel, gyro, and step count on screen.
void dispShowImuRaw( float ax, float ay, float az,
                     float gx, float gy, float gz,
                     uint32_t steps )
{
    tft.fillScreen( DISP_COLOR_BG );

    // Title
    tft.setTextColor( ST77XX_MAGENTA );
    tft.setTextSize( 2 );
    tft.setCursor( 4, 4 );
    tft.print( "IMU RAW TEST" );

    // Accel row
    tft.setTextColor( ST77XX_GREEN );
    tft.setTextSize( 1 );
    tft.setCursor( 4, 30 );
    tft.print( "Accel (g): X=" );
    tft.print( ax, 2 );
    tft.print( "  Y=" );
    tft.print( ay, 2 );
    tft.print( "  Z=" );
    tft.print( az, 2 );

    // Gyro row
    tft.setTextColor( ST77XX_YELLOW );
    tft.setTextSize( 1 );
    tft.setCursor( 4, 50 );
    tft.print( "Gyro (dps): X=" );
    tft.print( gx, 2 );
    tft.print( "  Y=" );
    tft.print( gy, 2 );
    tft.print( "  Z=" );
    tft.print( gz, 2 );

    // Steps
    tft.setTextColor( ST77XX_WHITE );
    tft.setTextSize( 2 );
    tft.setCursor( 4, 72 );
    tft.print( "STEPS: " );
    tft.print( steps );
}

// Puts the display into sleep mode and turns the backlight off.
void dispSleep()
{
    DBG( "DISP", "Entering sleep (backlight off)" );
    tft.enableSleep( true );
    digitalWrite( TFT_BACKLITE_PIN, LOW );
}

// Wakes the display from sleep and turns the backlight on.
void dispWake()
{
    DBG( "DISP", "Waking (backlight on)" );
    tft.enableSleep( false );
    digitalWrite( TFT_BACKLITE_PIN, HIGH );
    delay( 10 );  // brief stabilisation delay
}
