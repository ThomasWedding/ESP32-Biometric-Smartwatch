#include "bluetooth.h"
#include "dataBuffer.h"
#include "debug.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include <sys/time.h>

// UUIDs
static const char* BLE_SERVICE_UUID       = "4FAFC201-1FB5-459E-8FCC-C5C9C331914B";
static const char* BLE_CHAR_READINGS_UUID = "BEB54842-36E1-4688-B7F5-EA07361B26A8";

// Standard Device Information Service (0x180A)
static const char* DIS_SERVICE_UUID      = "180A";
static const char* DIS_MANUFACTURER_UUID = "2A29";
static const char* DIS_MODEL_UUID        = "2A24";
static const char* DIS_FIRMWARE_UUID     = "2A26";

// Standard Heart Rate Service (0x180D)
static const char* HRS_SERVICE_UUID       = "180D";
static const char* HRS_MEASUREMENT_UUID   = "2A37";  // Heart Rate Measurement (NOTIFY)
static const char* HRS_BODY_LOCATION_UUID = "2A38";  // Body Sensor Location (READ)

// Time synchronisation characteristics (on the custom bio service)
// Android writes an 8-byte little-endian uint64_t Unix timestamp (seconds, local time).
// The watch notifies BLE_CHAR_TIME_REQUEST_UUID to prompt Android to send the timestamp.
static const char* BLE_CHAR_TIME_WRITE_UUID   = "C1B2D3E4-F5A6-7890-BCDE-012345678901";
static const char* BLE_CHAR_TIME_REQUEST_UUID = "D1E2F3A4-B5C6-7890-CDEF-123456789012";

// Packed payload sent as a single NOTIFY per reading (19 bytes, little-endian)
struct __attribute__((packed)) BlePayload {
    uint64_t timestamp;      // Unix epoch ms (if timestampType=1) or millis() (if timestampType=0)
    uint16_t heartRate;      // BPM
    uint8_t  spo2;           // 0–100 %
    uint16_t hrv;            // RMSSD ms
    uint32_t stepCount;      // cumulative
    uint8_t  valid;          // 1 = valid reading
    uint8_t  timestampType;  // 0 = millis(), 1 = Unix epoch ms
};

static const char* DEVICE_NAME = "Biowatch";

// BLE object handles
static NimBLEServer*         bleServer           = nullptr;
static NimBLECharacteristic* bleReadingsChar      = nullptr;
static NimBLECharacteristic* bleHrMeasChar        = nullptr;
static NimBLECharacteristic* bleTimeWriteChar     = nullptr;
static NimBLECharacteristic* bleTimeRequestChar   = nullptr;
static bool bleConnected = false;

// True once Android has sent a valid timestamp; RTC_DATA_ATTR preserves the flag
// across deep sleep so the watch knows it has valid time when it wakes.
static RTC_DATA_ATTR bool     bleRtcSynced      = false;

// Anchor values saved at the moment of the first (and each subsequent) time sync.
// Used to back-convert pre-sync millis() timestamps to Unix ms for buffered readings.
// Both are RTC_DATA_ATTR so the anchor survives deep sleep alongside bleRtcSynced.
static RTC_DATA_ATTR uint64_t bleSyncEpochMs    = 0;   // Unix epoch ms at time of sync
static RTC_DATA_ATTR uint32_t bleSyncMillis      = 0;   // millis() at time of sync

// Maps HCI disconnect reason codes to human-readable strings.
static const char* bleHciReasonStr( int reason )
{
    // NimBLE encodes HCI pass-through errors as 0x200 + HCI code; strip the prefix.
    int hci = ( reason >= 0x200 ) ? ( reason - 0x200 ) : reason;
    switch( hci )
    {
        case 0x08: return "Connection timeout (supervision)";
        case 0x13: return "Remote user terminated";
        case 0x16: return "Local host terminated";
        case 0x22: return "Unacceptable connection parameters";
        case 0x28: return "Instant passed";
        case 0x3B: return "Connection failed to be established / parameter rejection";
        case 0x3D: return "Connection terminated (MIC failure)";
        case 0x3E: return "Connection failed to be established";
        default:   return "Unknown";
    }
}

// Server callbacks. Handle connect, disconnect, MTU change, and auth events.
class BioServerCallbacks : public NimBLEServerCallbacks
{
    void onConnect( NimBLEServer* pServer, NimBLEConnInfo& connInfo ) override
    {
        bleConnected = true;
        DBG( "BLE", "=== CONNECT === peer: %s  handle: %u",
            connInfo.getAddress().toString().c_str(),
            connInfo.getConnHandle() );
        DBG( "BLE", "  interval: %.2f ms  latency: %u  timeout: %u ms",
            connInfo.getConnInterval() * 1.25f,
            connInfo.getConnLatency(),
            connInfo.getConnTimeout() * 10 );
        DBG( "BLE", "  encrypted: %s  authenticated: %s  bonded: %s",
            connInfo.isEncrypted() ? "yes" : "no",
            connInfo.isAuthenticated() ? "yes" : "no",
            connInfo.isBonded() ? "yes" : "no" );
    }

    void onDisconnect( NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason ) override
    {
        bleConnected = false;
        int hciCode = ( reason >= 0x200 ) ? ( reason - 0x200 ) : reason;
        DBG( "BLE", "=== DISCONNECT === peer: %s  raw: 0x%03X  hci: 0x%02X (%s)",
            connInfo.getAddress().toString().c_str(),
            reason,
            hciCode,
            bleHciReasonStr( reason ) );
        DBG( "BLE", "  encrypted: %s  authenticated: %s  bonded: %s",
            connInfo.isEncrypted() ? "yes" : "no",
            connInfo.isAuthenticated() ? "yes" : "no",
            connInfo.isBonded() ? "yes" : "no" );
        DBG( "BLE", "Restarting advertising..." );
        NimBLEDevice::getAdvertising() -> start();
    }

    void onMTUChange( uint16_t MTU, NimBLEConnInfo& connInfo ) override
    {
        DBG( "BLE", "MTU changed - peer: %s  MTU: %u",
            connInfo.getAddress().toString().c_str(),
            MTU );
    }

    void onAuthenticationComplete( NimBLEConnInfo& connInfo ) override
    {
        DBG( "BLE", "=== AUTH COMPLETE === peer: %s",
            connInfo.getAddress().toString().c_str() );
        DBG( "BLE", "  encrypted: %s  authenticated: %s  bonded: %s",
            connInfo.isEncrypted() ? "yes" : "no",
            connInfo.isAuthenticated() ? "yes" : "no",
            connInfo.isBonded() ? "yes" : "no" );
        // Do NOT disconnect on unencrypted auth, let Android decide.
    }
};

// Handles writes to the time-sync characteristic.
// Expects an 8-byte little-endian uint64_t Unix timestamp (seconds, local time).
// Sets the ESP32 system clock via settimeofday(); the RTC timer persists this
// through light and deep sleep so gettimeofday() remains accurate after wakeup.
class BleTimeWriteCallbacks : public NimBLECharacteristicCallbacks
{
    void onWrite( NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo ) override
    {
        std::string val = pChar -> getValue();
        if( val.length() < 8 )
        {
            DBG( "BLE", "Time sync: short payload (%u bytes)", ( unsigned )val.length() );
            return;
        }

        // Decode little-endian uint64_t without memcpy
        uint64_t epoch = 0;
        const uint8_t* b = reinterpret_cast<const uint8_t*>( val.data() );
        for( int i = 0; i < 8; i++ )
        {
            epoch |= ( uint64_t )b[ i ] << ( i * 8 );
        }

        struct timeval tv;
        tv.tv_sec  = ( time_t )epoch;
        tv.tv_usec = 0;
        settimeofday( &tv, nullptr );

        // Save the sync anchor so pre-sync millis() timestamps in buffered readings
        // can be back-converted to Unix ms at flush time.
        bleSyncEpochMs = epoch * 1000ULL;
        bleSyncMillis  = ( uint32_t )millis();
        bleRtcSynced   = true;

        struct tm tmInfo;
        gmtime_r( &tv.tv_sec, &tmInfo );
        DBG( "BLE", "Time sync: %02d/%02d/%04d %02d:%02d:%02d (epoch=%llu)",
             tmInfo.tm_mon + 1, tmInfo.tm_mday, tmInfo.tm_year + 1900,
             tmInfo.tm_hour, tmInfo.tm_min, tmInfo.tm_sec,
             ( unsigned long long )epoch );
    }
};

// Initialises the NimBLE stack, GATT services, and starts advertising.
void bleInit()
{
    DBG( "BLE", "Initialising NimBLE stack..." );

    NimBLEDevice::init( DEVICE_NAME );

    // Clear any stale bond keys from NVS left by previous failed pairing attempts.
    DBG( "BLE", "Bonds in NVS before clear: %d", NimBLEDevice::getNumBonds() );
    NimBLEDevice::deleteAllBonds();
    DBG( "BLE", "Bonds in NVS after clear: %d", NimBLEDevice::getNumBonds() );

    NimBLEDevice::setMTU( 247 );
    NimBLEDevice::setPower( 0 );

    // BLE pairing for Android. SC=true is mandatory on Android 10+.
    NimBLEDevice::setSecurityAuth( true, false, true );             // bonding=true, MITM=false, SC=true
    NimBLEDevice::setSecurityIOCap( BLE_HS_IO_NO_INPUT_OUTPUT );    // "Just Works" pairing

    // Initialize BLE server
    bleServer = NimBLEDevice::createServer();
    bleServer -> setCallbacks( new BioServerCallbacks() );

    NimBLEService* bleBioService = bleServer -> createService( BLE_SERVICE_UUID );

    // Single packed readings characteristic. 19-byte BlePayload, NOTIFY + READ
    bleReadingsChar = bleBioService -> createCharacteristic(
        BLE_CHAR_READINGS_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );

    // Time-sync write characteristic. Android writes an 8-byte little-endian
    // uint64_t Unix timestamp (seconds, local time) here in response to a request.
    bleTimeWriteChar = bleBioService -> createCharacteristic(
        BLE_CHAR_TIME_WRITE_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    bleTimeWriteChar -> setCallbacks( new BleTimeWriteCallbacks() );

    // Time-sync request characteristic. Watch notifies here (value 0x01) to
    // prompt Android to write the current time to bleTimeWriteChar.
    bleTimeRequestChar = bleBioService -> createCharacteristic(
        BLE_CHAR_TIME_REQUEST_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );

    // Device Information Service. Standard BLE service that Android system
    NimBLEService* bleDisService = bleServer -> createService( DIS_SERVICE_UUID );
    bleDisService -> createCharacteristic( DIS_MANUFACTURER_UUID, NIMBLE_PROPERTY::READ )
        -> setValue( "Biowatch" );
    bleDisService -> createCharacteristic( DIS_MODEL_UUID, NIMBLE_PROPERTY::READ )
        -> setValue( "BW-1" );
    bleDisService -> createCharacteristic( DIS_FIRMWARE_UUID, NIMBLE_PROPERTY::READ )
        -> setValue( "0.1.0" );

    // Heart Rate Service. Standard BLE profile for Android
    NimBLEService* bleHrsService = bleServer -> createService( HRS_SERVICE_UUID );

    // Heart Rate Measurement (0x2A37) - BT SIG format:
    //   Byte 0: flags (0x06 = UINT8 HR format + sensor contact detected & supported)
    //   Byte 1: heart rate value (uint8)
    bleHrMeasChar = bleHrsService -> createCharacteristic(
        HRS_MEASUREMENT_UUID,
        NIMBLE_PROPERTY::NOTIFY
    );

    // Body Sensor Location (0x2A38). 0x02 = "Wrist"
    bleHrsService -> createCharacteristic( HRS_BODY_LOCATION_UUID, NIMBLE_PROPERTY::READ )
        -> setValue( ( uint8_t )0x02 );

    // Start BLE server
    bleServer -> start();

    NimBLEAdvertising* bleAdv = NimBLEDevice::getAdvertising();
    bleAdv -> setName( DEVICE_NAME );
    bleAdv -> addServiceUUID( HRS_SERVICE_UUID );
    bleAdv -> addServiceUUID( BLE_SERVICE_UUID );
    bleAdv -> addServiceUUID( DIS_SERVICE_UUID );
    bleAdv -> setAppearance( 0x00C0 );    // Generic Watch (BT SIG appearance value)
    bleAdv -> enableScanResponse( true );
    bleAdv -> setMinInterval( 800 );       // 500 ms (units of 0.625 ms)
    bleAdv -> setMaxInterval( 1600 );      // 1000 ms

    DBG( "BLE", "Starting advertising..." );
    bleAdv -> start();

    DBG( "BLE", "Ready - device name: %s", DEVICE_NAME );
}

// Returns true if a central is currently connected.
bool bleIsConnected()
{
    return bleConnected;
}

// Returns true if the RTC has been synced at least once via BLE time-sync.
bool bleIsTimeSynced()
{
    return bleRtcSynced;
}

// Returns the current time as Unix epoch milliseconds.
// Only call this after bleIsTimeSynced() returns true; result is undefined otherwise.
uint64_t bleGetUnixMs()
{
    struct timeval tv;
    gettimeofday( &tv, nullptr );
    return ( uint64_t )tv.tv_sec * 1000ULL + ( uint64_t )tv.tv_usec / 1000ULL;
}

// Notifies the time-request characteristic to prompt Android to send the current time.
// Android should respond by writing an 8-byte Unix timestamp to the time-write characteristic.
void bleRequestTimeSync()
{
    if( !bleConnected || !bleTimeRequestChar ) return;
    uint8_t req = 0x01;
    bleTimeRequestChar -> setValue( &req, sizeof( req ) );
    bleTimeRequestChar -> notify();
    DBG( "BLE", "Time sync request sent" );
}

// Reads the current time from the ESP32 RTC and populates the output parameters.
// Returns true if the RTC has been synced at least once via BLE; false means the
// caller should fall back to its own hardcoded clock values.
// The ESP32 RTC timer runs during both light and deep sleep, so the returned time
// stays accurate across sleep cycles after the initial sync.
bool bleGetCurrentTime( uint8_t* month, uint8_t* day, uint16_t* year,
                        uint8_t* hour, uint8_t* minute, uint8_t* second, bool* isPm )
{
    if( !bleRtcSynced ) return false;

    struct timeval tv;
    gettimeofday( &tv, nullptr );

    struct tm tmInfo;
    gmtime_r( &tv.tv_sec, &tmInfo );

    *month  = ( uint8_t )( tmInfo.tm_mon + 1 );
    *day    = ( uint8_t )tmInfo.tm_mday;
    *year   = ( uint16_t )( tmInfo.tm_year + 1900 );
    *minute = ( uint8_t )tmInfo.tm_min;
    *second = ( uint8_t )tmInfo.tm_sec;

    // Convert 24-hour to 12-hour with AM/PM
    uint8_t h24 = ( uint8_t )tmInfo.tm_hour;
    *isPm  = ( h24 >= 12 );
    *hour  = h24 % 12;
    if( *hour == 0 ) *hour = 12;

    return true;
}

// Packs a BiometricReading into a BlePayload and notifies the connected central.
// Also pushes HR data to the standard Heart Rate Measurement characteristic.
void bleSendReading( const BiometricReading& reading )
{
    if( !bleConnected ) return;

    BlePayload payload;
    payload.timestamp     = reading.timestamp;
    payload.heartRate     = reading.heartRate;
    payload.spo2          = reading.spo2;
    payload.hrv           = reading.hrv;
    payload.stepCount     = reading.stepCount;
    payload.valid         = reading.valid ? 1 : 0;
    payload.timestampType = reading.timestampIsUnix ? 1 : 0;

    bleReadingsChar -> setValue( reinterpret_cast<uint8_t*>( &payload ), sizeof( payload ) );
    bool bleSent = bleReadingsChar -> notify();
    if( bleSent )
    {
        DBG( "BLE", "Sent - ts=%llu (type=%s)  bpm=%u  spo2=%u  hrv=%u  steps=%lu",
            ( unsigned long long )reading.timestamp,
            reading.timestampIsUnix ? "unix" : "millis",
            reading.heartRate, reading.spo2,
            reading.hrv, ( unsigned long )reading.stepCount );
    } else {
        DBG( "BLE", "notify() failed - dropped reading ts=%llu", ( unsigned long long )reading.timestamp );
    }

    // Also update the standard Heart Rate Measurement characteristic.
    // Format per BT SIG Heart Rate Profile:
    //   Byte 0: flags - 0x06 = UINT8 HR value + sensor contact detected & supported
    //   Byte 1: heart rate (uint8, capped at 255)
    uint8_t hrPacket[ 2 ] = {
        0x06,
        ( uint8_t )min( ( uint16_t )255, reading.heartRate )
    };
    bleHrMeasChar -> setValue( hrPacket, sizeof( hrPacket ) );
    bleHrMeasChar -> notify();
}

// Drains all buffered readings and sends them to the connected central.
// If time is synced, any pre-sync millis() timestamps are back-converted to Unix ms
// using the anchor saved at sync time before the reading is transmitted.
void bleFlushBuffer()
{
    BiometricReading reading;
    while( bufPop( reading ) )
    {
        if( bleRtcSynced && !reading.timestampIsUnix )
        {
            // Convert millis() timestamp to Unix ms. The difference can be negative
            // (reading taken before sync) or positive (reading taken after sync but
            // before the first flush), both handled correctly by signed arithmetic.
            int64_t offsetMs      = ( int64_t )reading.timestamp - ( int64_t )bleSyncMillis;
            reading.timestamp     = ( uint64_t )( ( int64_t )bleSyncEpochMs + offsetMs );
            reading.timestampIsUnix = true;
        }
        bleSendReading( reading );
    }
}

// Flushes the buffer if connected, time-synced, and readings are pending.
// Holds the buffer until the first time sync completes so Android never
// receives a millis()-based timestamp. Called periodically by bleTask.
void bleProcess()
{
    if( bleConnected && bleRtcSynced && !bufIsEmpty() )
    {
        bleFlushBuffer();
    }
}
