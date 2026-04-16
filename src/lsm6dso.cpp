#include "lsm6dso.h"
#include "debug.h"
#include <LSM6DSOSensor.h>

// Static sensor instance; I2C 7-bit address 0x6B (SA0 pin high).
// The library IO layer right-shifts the stored address (expects 8-bit format),
// so we pass 0x6B << 1 = 0xD6 to get the correct 7-bit address on the wire.
LSM6DSOSensor imuSensor( &Wire, 0xD6 );

// 7-bit I2C address
static const uint8_t IMU_I2C_ADDR = 0x6B;

// Register addresses for embedded function page access
static const uint8_t REG_FUNC_CFG_ACCESS = 0x01;
static const uint8_t REG_PAGE_SEL        = 0x02;
static const uint8_t REG_PAGE_ADDRESS    = 0x08;
static const uint8_t REG_PAGE_VALUE      = 0x09;
static const uint8_t REG_PAGE_RW         = 0x17;

// Embedded function register address for pedometer debounce
static const uint16_t REG_PEDO_DEB_STEPS_CONF = 0x184;

// I2C helpers for embedded function page registers.
// The LSM6DSO's pedometer debounce register is in an embedded function
// memory page that requires a bank-switch + page-address sequence to access.
// Since reg_ctx is private in LSM6DSOSensor, the page-access protocol from
// lsm6dso_reg.c is replicated here using direct Wire reads/writes.
static void imuWriteReg( uint8_t reg, uint8_t val )
{
    Wire.beginTransmission( IMU_I2C_ADDR );
    Wire.write( reg );
    Wire.write( val );
    Wire.endTransmission();
}

static uint8_t imuReadReg( uint8_t reg )
{
    Wire.beginTransmission( IMU_I2C_ADDR );
    Wire.write( reg );
    Wire.endTransmission( false );
    Wire.requestFrom( IMU_I2C_ADDR, ( uint8_t )1 );
    return Wire.read();
}

// Reads a value from the embedded function memory page at the given 16-bit address.
static bool imuReadEmbeddedFuncReg( uint16_t addr, uint8_t* val )
{
    // Switch to embedded function bank
    uint8_t funcCfg = imuReadReg( REG_FUNC_CFG_ACCESS );
    imuWriteReg( REG_FUNC_CFG_ACCESS, ( funcCfg & 0x3F ) | 0x80 );

    // Enable page read
    uint8_t pageRw = imuReadReg( REG_PAGE_RW );
    pageRw = ( pageRw & 0x9F ) | ( 0x01 << 5 );  // page_rw = 0x01 (read)
    imuWriteReg( REG_PAGE_RW, pageRw );

    // Set page select: page_sel [7:4] = upper nibble of address,
    // not_used_01 [3:0] = 0x01 (required by the chip)
    uint8_t pageSel = ( ( ( uint8_t )( addr >> 8 ) & 0x0F ) << 4 ) | 0x01;
    imuWriteReg( REG_PAGE_SEL, pageSel );

    // Set page address (lower byte)
    imuWriteReg( REG_PAGE_ADDRESS, ( uint8_t )( addr & 0xFF ) );

    // Read the value
    *val = imuReadReg( REG_PAGE_VALUE );

    // Disable page read
    pageRw = imuReadReg( REG_PAGE_RW );
    pageRw &= 0x9F;  // page_rw = 0x00
    imuWriteReg( REG_PAGE_RW, pageRw );

    // Switch back to user bank
    funcCfg = imuReadReg( REG_FUNC_CFG_ACCESS );
    imuWriteReg( REG_FUNC_CFG_ACCESS, funcCfg & 0x3F );

    return true;
}

// Writes a value to the embedded function memory page at the given 16-bit address.
static bool imuWriteEmbeddedFuncReg( uint16_t addr, uint8_t val )
{
    // Switch to embedded function bank
    uint8_t funcCfg = imuReadReg( REG_FUNC_CFG_ACCESS );
    imuWriteReg( REG_FUNC_CFG_ACCESS, ( funcCfg & 0x3F ) | 0x80 );

    // Enable page write
    uint8_t pageRw = imuReadReg( REG_PAGE_RW );
    pageRw = ( pageRw & 0x9F ) | ( 0x02 << 5 );  // page_rw = 0x02 (write)
    imuWriteReg( REG_PAGE_RW, pageRw );

    // Set page select: page_sel [7:4] = upper nibble of address,
    // not_used_01 [3:0] = 0x01 (required by the chip)
    uint8_t pageSel = ( ( ( uint8_t )( addr >> 8 ) & 0x0F ) << 4 ) | 0x01;
    imuWriteReg( REG_PAGE_SEL, pageSel );

    // Set page address (lower byte)
    imuWriteReg( REG_PAGE_ADDRESS, ( uint8_t )( addr & 0xFF ) );

    // Write the value
    imuWriteReg( REG_PAGE_VALUE, val );

    // Disable page write
    pageRw = imuReadReg( REG_PAGE_RW );
    pageRw &= 0x9F;  // page_rw = 0x00
    imuWriteReg( REG_PAGE_RW, pageRw );

    // Switch back to user bank
    funcCfg = imuReadReg( REG_FUNC_CFG_ACCESS );
    imuWriteReg( REG_FUNC_CFG_ACCESS, funcCfg & 0x3F );

    return true;
}

// Initialises the LSM6DSO and enables accelerometer, gyroscope, and pedometer.
// pedometerDebounceSteps sets the number of consecutive steps before counting begins.
void imuInit( uint8_t pedometerDebounceSteps )
{
    if( imuSensor.begin() != LSM6DSO_OK )
        DBG( "IMU", "ERROR: begin() failed - check I2C wiring/address" );

    // Enable accelerometer at 26 Hz, ±2g full-scale
    if( imuSensor.Enable_X() != LSM6DSO_OK )
        DBG( "IMU", "ERROR: Enable_X() (accelerometer) failed" );

    // Enable gyroscope at 26 Hz, ±250 dps full-scale
    if( imuSensor.Enable_G() != LSM6DSO_OK )
        DBG( "IMU", "ERROR: Enable_G() (gyroscope) failed" );

    // Enable pedometer (step counter)
    if( imuSensor.Enable_Pedometer() != LSM6DSO_OK )
        DBG( "IMU", "ERROR: Enable_Pedometer() failed" );

    // Lower the pedometer debounce threshold from the chip default (~11 steps)
    uint8_t defaultDebounce = 0;
    imuReadEmbeddedFuncReg( REG_PEDO_DEB_STEPS_CONF, &defaultDebounce );
    DBG( "IMU", "Pedometer debounce default = %u steps", defaultDebounce );
    imuWriteEmbeddedFuncReg( REG_PEDO_DEB_STEPS_CONF, pedometerDebounceSteps );

    uint8_t verifyDebounce = 0;
    imuReadEmbeddedFuncReg( REG_PEDO_DEB_STEPS_CONF, &verifyDebounce );
    DBG( "IMU", "Pedometer debounce set to %u steps", verifyDebounce );

    // Read WHO_AM_I to verify chip identity
    uint8_t whoAmI = 0;
    imuSensor.ReadID( &whoAmI );
    DBG( "IMU", "WHO_AM_I=0x%02X (expected 0x6C)", whoAmI );
    DBG( "IMU", "Init complete - accel, gyro, pedometer enabled" );
}

// Reads the current cumulative step count from the embedded pedometer into steps.
bool imuGetStepCount( uint32_t* steps )
{
    if( steps == nullptr )
        return false;
    uint16_t rawSteps = 0;
    if( imuSensor.Get_Step_Count( &rawSteps ) != LSM6DSO_OK )
    {
        DBG( "IMU", "Get_Step_Count() failed" );
        return false;
    }
    *steps = ( uint32_t )rawSteps;
    return true;
}

// Resets the embedded pedometer step counter to zero.
void imuResetStepCount()
{
    DBG( "IMU", "Resetting step counter" );
    imuSensor.Step_Counter_Reset();
}

// Reads the current 3-axis acceleration in g into ax, ay, az.
bool imuGetAcceleration( float* ax, float* ay, float* az )
{
    if( ax == nullptr || ay == nullptr || az == nullptr )
        return false;

    int32_t rawAxes[ 3 ] = { 0, 0, 0 };
    if( imuSensor.Get_X_Axes( rawAxes ) != LSM6DSO_OK )
    {
        DBG( "IMU", "Get_X_Axes() (accel) failed" );
        return false;
    }

    // Raw values are in mg (milli-g); divide by 1000 for g
    *ax = ( float )rawAxes[ 0 ] / 1000.0f;
    *ay = ( float )rawAxes[ 1 ] / 1000.0f;
    *az = ( float )rawAxes[ 2 ] / 1000.0f;
    return true;
}

// Reads the current 3-axis angular rate in dps into gx, gy, gz.
bool imuGetAngularRate( float* gx, float* gy, float* gz )
{
    if( gx == nullptr || gy == nullptr || gz == nullptr )
        return false;

    int32_t rawAxes[ 3 ] = { 0, 0, 0 };
    if( imuSensor.Get_G_Axes( rawAxes ) != LSM6DSO_OK )
    {
        DBG( "IMU", "Get_G_Axes() (gyro) failed" );
        return false;
    }
    // Raw values are in mdps (milli-dps); divide by 1000 for dps
    *gx = ( float )rawAxes[ 0 ] / 1000.0f;
    *gy = ( float )rawAxes[ 1 ] / 1000.0f;
    *gz = ( float )rawAxes[ 2 ] / 1000.0f;
    return true;
}

// Enables the LSM6DSO embedded tilt algorithm, routed to INT1.
bool imuEnableTiltDetection()
{
    if( imuSensor.Enable_Tilt_Detection( LSM6DSO_INT1_PIN ) != LSM6DSO_OK )
    {
        DBG( "IMU", "ERROR: Enable_Tilt_Detection(INT1) failed" );
        return false;
    }
    DBG( "IMU", "Tilt detection enabled on INT1" );
    return true;
}

// Returns true if a tilt event has been detected since the last read.
bool imuCheckTiltEvent()
{
    LSM6DSO_Event_Status_t status;
    if( imuSensor.Get_X_Event_Status( &status ) != LSM6DSO_OK )
        return false;
    return status.TiltStatus != 0;
}

// Sets the gyroscope ODR to 0 (power-down mode). The accelerometer, pedometer,
// and tilt detection continue running unaffected.
void imuDisableGyro()
{
    if( imuSensor.Disable_G() != LSM6DSO_OK )
        DBG( "IMU", "ERROR: Disable_G() failed" );
    else
        DBG( "IMU", "Gyroscope disabled" );
}

// Re-enables the gyroscope at its previously configured ODR (26 Hz, ±250 dps).
void imuEnableGyro()
{
    if( imuSensor.Enable_G() != LSM6DSO_OK )
        DBG( "IMU", "ERROR: Enable_G() failed" );
    else
        DBG( "IMU", "Gyroscope enabled" );
}

// FSM WRIST-TILT IMPLEMENTATION (disabled — axis mapping unverified for this mounting)
// Source: ST MEMS FSM repository, examples/wrist_tilt_detection_xl_gy/lsm6dso (right wrist)
// To re-enable: uncomment, declare in lsm6dso.h, call instead of imuEnableTiltDetection()
// in setup(), remove imuDisableGyro/imuEnableGyro from powerManager.cpp, and set
// WAKE_FSM_ONLY in main.cpp once gesture detection is validated on this hardware.
//
// Note: the FSM config zeros EMB_FUNC_EN_A (disabling the pedometer). After loading,
// restore PEDO_EN with direct register writes rather than Enable_Pedometer() to avoid
// algorithm re-initialization side effects:
//   imuWriteReg( REG_FUNC_CFG_ACCESS, 0x80 );
//   imuWriteReg( 0x04, 0x08 );   // EMB_FUNC_EN_A: PEDO_EN = bit 3
//   imuWriteReg( REG_FUNC_CFG_ACCESS, 0x00 );
//
// bool imuEnableFsmWristTilt()
// {
//     static const uint8_t IMU_FSM_CONFIG[][ 2 ] = {
//         { 0x10, 0x00 }, { 0x11, 0x00 }, { 0x01, 0x80 },
//         { 0x04, 0x00 }, { 0x05, 0x00 }, { 0x5F, 0x53 },
//         { 0x46, 0x01 }, { 0x47, 0x00 }, { 0x0A, 0x00 },
//         { 0x0B, 0x01 }, { 0x0C, 0x00 }, { 0x0E, 0x00 },
//         { 0x0F, 0x00 }, { 0x10, 0x00 }, { 0x17, 0x40 },
//         { 0x09, 0x00 }, { 0x02, 0x11 }, { 0x08, 0x7A },
//         { 0x09, 0x00 }, { 0x09, 0x00 }, { 0x09, 0x01 },
//         { 0x09, 0x01 }, { 0x09, 0x00 }, { 0x09, 0x04 },
//         { 0x02, 0x41 }, { 0x08, 0x00 },
//         { 0x09, 0xB1 }, { 0x09, 0x30 }, { 0x09, 0x34 },
//         { 0x09, 0x00 }, { 0x09, 0x1D }, { 0x09, 0x00 },
//         { 0x09, 0x96 }, { 0x09, 0x31 }, { 0x09, 0x00 },
//         { 0x09, 0xB8 }, { 0x09, 0x80 }, { 0x09, 0x00 },
//         { 0x09, 0x20 }, { 0x09, 0x00 }, { 0x09, 0x40 },
//         { 0x09, 0x00 }, { 0x09, 0xEC }, { 0x09, 0x24 },
//         { 0x09, 0x00 }, { 0x09, 0x00 }, { 0x09, 0x00 },
//         { 0x09, 0x00 }, { 0x09, 0x00 }, { 0x09, 0x00 },
//         { 0x09, 0x00 }, { 0x09, 0x00 }, { 0x09, 0x00 },
//         { 0x09, 0x08 }, { 0x09, 0x00 }, { 0x09, 0x66 },
//         { 0x09, 0xAA }, { 0x09, 0x96 }, { 0x09, 0x31 },
//         { 0x09, 0x23 }, { 0x09, 0x01 }, { 0x09, 0xA5 },
//         { 0x09, 0xFF }, { 0x09, 0x42 }, { 0x09, 0x0A },
//         { 0x09, 0xAA }, { 0x09, 0x96 }, { 0x09, 0x39 },
//         { 0x09, 0x23 }, { 0x09, 0x07 }, { 0x09, 0xA5 },
//         { 0x09, 0x23 }, { 0x09, 0x00 }, { 0x09, 0x77 },
//         { 0x09, 0x83 }, { 0x09, 0x88 }, { 0x09, 0x86 },
//         { 0x09, 0x22 },
//         { 0x04, 0x00 }, { 0x05, 0x01 }, { 0x17, 0x00 },
//         { 0x01, 0x00 }, { 0x01, 0x00 }, { 0x02, 0x3F },
//         { 0x07, 0x00 }, { 0x08, 0x00 }, { 0x09, 0x00 },
//         { 0x0A, 0x00 }, { 0x0B, 0x00 }, { 0x0C, 0x00 },
//         { 0x0D, 0x00 }, { 0x0E, 0x00 },
//         { 0x10, 0x3C }, { 0x11, 0x3C }, { 0x12, 0x04 },
//         { 0x13, 0x00 }, { 0x14, 0x00 }, { 0x15, 0x10 },
//         { 0x16, 0x80 }, { 0x17, 0x00 }, { 0x18, 0xE0 },
//         { 0x19, 0x00 },
//         { 0x56, 0x00 }, { 0x57, 0x00 }, { 0x58, 0x00 },
//         { 0x59, 0x00 }, { 0x5A, 0x00 }, { 0x5B, 0x00 },
//         { 0x5C, 0x00 }, { 0x5D, 0x00 }, { 0x5E, 0x02 },
//         { 0x5F, 0x00 },
//         { 0x6F, 0x00 }, { 0x70, 0x00 }, { 0x71, 0x00 },
//         { 0x72, 0x00 }, { 0x73, 0x00 }, { 0x74, 0x00 },
//         { 0x75, 0x00 },
//     };
//     static const uint16_t IMU_FSM_CONFIG_LEN =
//         sizeof( IMU_FSM_CONFIG ) / sizeof( IMU_FSM_CONFIG[ 0 ] );
//     for( uint16_t i = 0; i < IMU_FSM_CONFIG_LEN; i++ )
//         imuWriteReg( IMU_FSM_CONFIG[ i ][ 0 ], IMU_FSM_CONFIG[ i ][ 1 ] );
//     imuWriteReg( REG_FUNC_CFG_ACCESS, 0x80 );
//     imuWriteReg( 0x04, 0x08 );
//     imuWriteReg( REG_FUNC_CFG_ACCESS, 0x00 );
//     DBG( "IMU", "FSM wrist-tilt loaded (%u writes), pedometer re-enabled, INT1 armed",
//         IMU_FSM_CONFIG_LEN );
//     return true;
// }
