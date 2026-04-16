#include "dataBuffer.h"
#include "debug.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static BiometricReading bufRing[ DATA_BUFFER_CAPACITY ];
static uint16_t         bufHead  = 0;
static uint16_t         bufTail  = 0;
static uint16_t         bufItems = 0;
static portMUX_TYPE     bufMux   = portMUX_INITIALIZER_UNLOCKED;

// Initialises the ring buffer, resetting all indices and item count.
void bufInit()
{
    portENTER_CRITICAL( &bufMux );
    bufHead  = 0;
    bufTail  = 0;
    bufItems = 0;
    portEXIT_CRITICAL( &bufMux );
    DBG( "BUF", "Ring buffer initialised (capacity=%u)", DATA_BUFFER_CAPACITY );
}

// Pushes a reading onto the tail of the buffer. Returns false if the buffer is full.
bool bufPush( const BiometricReading& reading )
{
    portENTER_CRITICAL( &bufMux );
    if( bufItems >= DATA_BUFFER_CAPACITY )
    {
        portEXIT_CRITICAL( &bufMux );
        DBG( "BUF", "FULL - push rejected (count=%u)", DATA_BUFFER_CAPACITY );
        return false;
    }
    bufRing[ bufTail ] = reading;
    bufTail = ( bufTail + 1 ) % DATA_BUFFER_CAPACITY;
    bufItems++;
    uint16_t cnt = bufItems;  // snapshot inside critical section
    portEXIT_CRITICAL( &bufMux );
    DBG( "BUF", "Pushed reading (t=%llu unix=%d), count=%u/%u",
        ( unsigned long long )reading.timestamp, reading.timestampIsUnix ? 1 : 0, cnt, DATA_BUFFER_CAPACITY );
    return true;
}

// Pops the oldest reading from the head of the buffer. Returns false if empty.
bool bufPop( BiometricReading& reading )
{
    portENTER_CRITICAL( &bufMux );
    if( bufItems == 0 )
    {
        portEXIT_CRITICAL( &bufMux );
        return false;
    }
    reading = bufRing[ bufHead ];
    bufHead  = ( bufHead + 1 ) % DATA_BUFFER_CAPACITY;
    bufItems--;
    portEXIT_CRITICAL( &bufMux );
    return true;
}

// Reads the oldest reading without removing it. Returns false if empty.
bool bufPeek( BiometricReading& reading )
{
    portENTER_CRITICAL( &bufMux );
    if( bufItems == 0 )
    {
        portEXIT_CRITICAL( &bufMux );
        return false;
    }
    reading = bufRing[ bufHead ];
    portEXIT_CRITICAL( &bufMux );
    return true;
}

// Returns the number of readings currently in the buffer.
uint16_t bufGetCount()
{
    portENTER_CRITICAL( &bufMux );
    uint16_t count = bufItems;
    portEXIT_CRITICAL( &bufMux );
    return count;
}

// Returns true if the buffer contains no readings.
bool bufIsEmpty()
{
    return bufGetCount() == 0;
}

// Returns true if the buffer is at capacity.
bool bufIsFull()
{
    return bufGetCount() >= DATA_BUFFER_CAPACITY;
}

// Discards all readings and resets the buffer to empty.
void bufClear()
{
    portENTER_CRITICAL( &bufMux );
    uint16_t was = bufItems;
    bufHead  = 0;
    bufTail  = 0;
    bufItems = 0;
    portEXIT_CRITICAL( &bufMux );
    DBG( "BUF", "Cleared (%u readings discarded)", was );
}
