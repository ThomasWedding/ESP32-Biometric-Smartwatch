#include "as7038rb.h"
#include "debug.h"
#include <Arduino.h>
#include <math.h>

// AS7038RB I2C address (fixed, no address-select pin).
// 7-bit address used by Arduino Wire (Wire shifts left internally to produce the 8-bit 0x60 on the bus).
static const uint8_t AS7038RB_I2C_ADDR = 0x30;

// Register map - addresses from AS7038RB datasheet DS000726 v2-00, section 6
// Core control / ID
static const uint8_t REG_CONTROL = 0x00; // ldo_en D0, osc_en D1, clk_def D2, hs_en D4
static const uint8_t REG_SUBID   = 0x91; // subid[4:0], Revision[2:0]
static const uint8_t REG_ID      = 0x92; // id[5:0]

// LED configuration
static const uint8_t REG_LED_CFG    = 0x10; // led1_en D0 … led4_en D3, sigref_en D7
static const uint8_t REG_LED1_CURRL = 0x12; // Curr1[1:0] in D7:D6, cs1_boost D0
static const uint8_t REG_LED1_CURRH = 0x13; // Curr1[9:2] in D7:D0
static const uint8_t REG_LED2_CURRL = 0x14; // Curr2[1:0] in D7:D6, cs2_boost D0
static const uint8_t REG_LED2_CURRH = 0x15; // Curr2[9:2] in D7:D0
static const uint8_t REG_LED3_CURRL = 0x16; // Curr3[1:0] in D7:D6, cs3_boost D0
static const uint8_t REG_LED3_CURRH = 0x17; // Curr3[9:2] in D7:D0
static const uint8_t REG_LED12_MODE = 0x2C; // LED 1 & 2 sequencer mode
static const uint8_t REG_LED34_MODE = 0x2D; // LED 3 & 4 sequencer mode

// Photodiode / amplifier
static const uint8_t REG_PD_CFG      = 0x1A; // pd_i0 D0, pd_i1 D1, pd1 D2, pd2 D3, pd3 D4, pd4 D5, pd_boost D6
static const uint8_t REG_PD_AMPRCCFG = 0x1D; // pd_ampres[2:0] D7:D5, pd_ampcap[4:0] D4:D0
static const uint8_t REG_PD_AMPCFG   = 0x1E; // pd_amp_en D7, pd_amp_auto D6, pd_ampvo[3:0] D5:D2, pd_ampcomp[1:0] D1:D0

// Optical front end
static const uint8_t REG_OFE_CFGA  = 0x50; // ofe2_en D7, ofe1_en D6, en_bias_ofe D5, aa_freq[1:0] D4:D3, gain_sd[2:0] D2:D0
static const uint8_t REG_OFE1_CFGA = 0x54;
static const uint8_t REG_OFE1_CFGB = 0x55;

// Sequencer
static const uint8_t REG_SEQ_CNT     = 0x30; // number of sequences (0 = run once)
static const uint8_t REG_SEQ_DIV     = 0x31; // clock divider
static const uint8_t REG_SEQ_START   = 0x32; // seq_start D0, seq_start_sync D1, seq_start_gpio D2
static const uint8_t REG_SEQ_PER     = 0x33; // sequence period
static const uint8_t REG_SEQ_LED_STA = 0x34; // LED on start time within sequence
static const uint8_t REG_SEQ_LED_STO = 0x35; // LED on stop time within sequence
static const uint8_t REG_SEQ_ADC     = 0x42; // ADC sampling time within sequence

// ADC
static const uint8_t REG_ADC_CFGA             = 0x88; // adc_multi_n[2:0] D2:D0, adc_multi_imode D3 (wait, D0)
static const uint8_t REG_ADC_CFGB             = 0x89; // adc_en D0, ulp D1, adc_calibration D2, adc_clock[2:0] D5:D3
static const uint8_t REG_ADC_CFGC             = 0x8A; // adc_settling_time[2:0] D2:D0, adc_discharge D3, adc_self_pd D4
static const uint8_t REG_ADC_CHANNEL_MASK_L   = 0x8B; // tia D0, ofe1 D1, sd1 D2, ofe2 D3, sd2 D4, temp D5, afe D6, pregain D7
static const uint8_t REG_ADC_CHANNEL_MASK_H   = 0x8C; // ecgo D0, ecgi D1, gpio2 D2, gpio3 D3
static const uint8_t REG_ADC_DATA_L           = 0x8E; // adc_data[7:0]
static const uint8_t REG_ADC_DATA_H           = 0x8F; // adc_data[13:8] in D5:D0

// FIFO
// Note: FIFO depth is 127 entries maximum (FIFO_CFG threshold field is 7 bits wide).
// At 100 Hz with red+IR+green interleaved (3 entries per sample) the FIFO holds ~42 complete
// PPG samples (~0.42 s). stopSampling drains whatever is available; for longer windows
// the FIFO must be drained periodically during acquisition (future enhancement).
static const uint8_t REG_FIFO_CFG   = 0x78; // fifo_threshold[6:0]
static const uint8_t REG_FIFO_CNTRL = 0x79; // fifo_clear D0
static const uint8_t REG_FIFOL      = 0xFE; // Fifol[7:0] - lower 8 bits of 14-bit FIFO sample
static const uint8_t REG_FIFOH      = 0xFF; // Fifoh[7:0] - upper bits; sample[13:8] in D5:D0

// Status / interrupts
static const uint8_t REG_STATUS    = 0xA0; // irq_adc D0, irq_sequencer D1, irq_ltf D2, irq_adc_threshold D3, irq_fifothreshold D4, irq_fifooverflow D5, irq_clipdetect D6, irq_led_supply_low D7
static const uint8_t REG_FIFOSTATUS = 0xA4; // fifooverflow D0
static const uint8_t REG_FIFOLEVEL  = 0xA6; // fifolevel[7:0]
static const uint8_t REG_INTENAB    = 0xA8;

// CONTROL register bit masks (REG_CONTROL 0x00)
static const uint8_t CTRL_LDO_EN = 0x01; // D0 - enable internal LDO
static const uint8_t CTRL_OSC_EN = 0x02; // D1 - enable internal oscillator

// LED_CFG register bit masks (REG_LED_CFG 0x10)
static const uint8_t LED_CFG_LED1_EN   = 0x01; // D0 - enable LED1 (red ~660 nm)
static const uint8_t LED_CFG_LED2_EN   = 0x02; // D1 - enable LED2 (IR ~940 nm)
static const uint8_t LED_CFG_LED3_EN   = 0x04; // D2 - enable LED3 (green ~520 nm)
static const uint8_t LED_CFG_SIGREF_EN = 0x80; // D7 - enable signal reference

// LED current (10-bit value; max 100 mA with csx_boost=0)
// ~10 mA = (10/100)*1023 = 102 = 0x066
// Write Curr[9:2] to CURRH, (Curr[1:0] << 6) to CURRL
static const uint16_t LED_CURRENT_10MA = 102;

// PD_CFG register bit masks (REG_PD_CFG 0x1A)
static const uint8_t PD_CFG_PD1 = 0x04; // D2
static const uint8_t PD_CFG_PD2 = 0x08; // D3

// PD_AMPCFG register bit masks (REG_PD_AMPCFG 0x1E)
static const uint8_t PD_AMP_EN = 0x80; // D7 - enable TIA amplifier

// OFE_CFGA register bit masks (REG_OFE_CFGA 0x50)
// Both OFE channels are active for all three LEDs; the sequencer fires LEDs in separate time
// slots and the shared OFE captures reflected light for each slot in turn.
static const uint8_t OFE_CFG_OFE1_EN = 0x40; // D6 - enable optical front end 1
static const uint8_t OFE_CFG_OFE2_EN = 0x80; // D7 - enable optical front end 2
static const uint8_t OFE_CFG_BIAS_EN = 0x20; // D5 - enable OFE bias

// ADC_CFGB register bit masks (REG_ADC_CFGB 0x89)
static const uint8_t ADC_CFGB_ADC_EN = 0x01; // D0 - enable ADC

// ADC channel mask bits (REG_ADC_CHANNEL_MASK_L 0x8B)
static const uint8_t ADC_MASK_OFE1 = 0x02; // D1 - capture OFE1
static const uint8_t ADC_MASK_OFE2 = 0x08; // D3 - capture OFE2

// MAN_SEQ_CFG register (REG_MAN_SEQ_CFG 0x2E)
static const uint8_t REG_MAN_SEQ_CFG    = 0x2E; // man_seq_cfg: seq_en D0
static const uint8_t MAN_SEQ_CFG_SEQ_EN = 0x01; // D0 - enable sequencer (default 0 = disabled)

// LED12_MODE sequencer fire modes for led1 (D2:D0) and led2 (D5:D3).
// LED34_MODE sequencer fire mode for led3 (D2:D0).
// 010 = fire every sequence; 000 = always off.
static const uint8_t LED1_MODE_SEQUENCER = 0x02; // led1_mode=010 in D2:D0 of LED12_MODE
static const uint8_t LED2_MODE_SEQUENCER = 0x10; // led2_mode=010 in D5:D3 of LED12_MODE (010 << 3)
static const uint8_t LED3_MODE_SEQUENCER = 0x02; // led3_mode=010 in D2:D0 of LED34_MODE

// FIFO drain interval used during multi-pass sampling.
// FIFO max depth is 127 entries; at 100 Hz (1 OFE channel) it fills in ~1.27 s.
// Draining every 800 ms keeps the FIFO comfortably below overflow.
static const uint32_t PPG_DRAIN_INTERVAL_MS = 800;

// SEQ_START register bit masks (REG_SEQ_START 0x32)
static const uint8_t SEQ_START_BIT = 0x01; // D0

// PPG ring buffers - red and IR for SpO2; green for heart rate and HRV
static const uint16_t PPG_BUFFER_SIZE = 300;

static uint32_t ppgRedBuffer[   PPG_BUFFER_SIZE ];
static uint32_t ppgIrBuffer[    PPG_BUFFER_SIZE ];
static uint32_t ppgGreenBuffer[ PPG_BUFFER_SIZE ];
static uint16_t ppgWriteIndex  = 0;
static uint16_t ppgSampleCount = 0;

// Computed results (populated after ppgStopSampling())
static uint16_t ppgHeartRate   = 0;
static uint8_t  ppgSpO2        = 0;
static uint16_t ppgHrv         = 0;
static bool     ppgResultsValid = false;

static TwoWire* ppgI2cBus = nullptr;

// Low-level I2C write to a single register.
static void ppgWriteReg( uint8_t reg, uint8_t value )
{
    ppgI2cBus -> beginTransmission( AS7038RB_I2C_ADDR );
    ppgI2cBus -> write( reg );
    ppgI2cBus -> write( value );
    uint8_t err = ppgI2cBus -> endTransmission();
    if( err != 0 )
        DBG( "PPG", "I2C write FAIL reg=0x%02X val=0x%02X err=%u", reg, value, err );
}

// Low-level I2C read from a single register; returns 0 on error.
static uint8_t ppgReadReg( uint8_t reg )
{
    ppgI2cBus -> beginTransmission( AS7038RB_I2C_ADDR );
    ppgI2cBus -> write( reg );
    uint8_t err = ppgI2cBus -> endTransmission( false );
    if( err != 0 )
    {
        DBG( "PPG", "I2C read FAIL reg=0x%02X err=%u", reg, err );
        return 0;
    }
    ppgI2cBus -> requestFrom( AS7038RB_I2C_ADDR, ( uint8_t )1 );
    if( ppgI2cBus -> available() )
        return ppgI2cBus -> read();
    DBG( "PPG", "I2C read EMPTY reg=0x%02X (no bytes returned)", reg );
    return 0;
}

// Reads two registers and combines them into a 16-bit value (high byte first).
static uint16_t ppgReadReg16( uint8_t highReg, uint8_t lowReg )
{
    uint16_t high = ppgReadReg( highReg );
    uint16_t low  = ppgReadReg( lowReg );
    return ( uint16_t )( ( high << 8 ) | low );
}

// Reads all available FIFO entries into buffer starting at *count; updates *count.
// Silently stops filling when *count reaches maxCount; returns number of entries drained.
static uint16_t ppgDrainFifoInto( uint32_t* buffer, uint16_t* count, uint16_t maxCount )
{
    uint8_t  level   = ppgReadReg( REG_FIFOLEVEL );
    uint16_t drained = 0;
    for( uint8_t i = 0; i < level && *count < maxCount; i++ )
    {
        uint8_t  lo             = ppgReadReg( REG_FIFOL );
        uint8_t  hi             = ppgReadReg( REG_FIFOH ) & 0x3F;
        buffer[ (*count)++ ]    = ( uint32_t )( ( ( uint16_t )hi << 8 ) | lo );
        drained++;
    }
    return drained;
}

// Activates one LED, runs the sequencer for durationMs with periodic FIFO draining, then stops.
// ledCfgBits is OR'd with LED_CFG_SIGREF_EN; ledMode12/34 set LED12_MODE and LED34_MODE.
// Accumulated samples are appended to buffer starting at *count.
static void ppgRunPass( uint8_t ledCfgBits, uint8_t ledMode12, uint8_t ledMode34,
                        uint32_t* buffer, uint16_t* count, uint16_t maxCount,
                        uint32_t durationMs )
{
    // Switch to this LED; all sequencer timing registers remain from ppgConfigure()
    ppgWriteReg( REG_LED_CFG,    LED_CFG_SIGREF_EN | ledCfgBits );
    ppgWriteReg( REG_LED12_MODE, ledMode12 );
    ppgWriteReg( REG_LED34_MODE, ledMode34 );

    // Clear FIFO and start
    ppgWriteReg( REG_FIFO_CNTRL, 0x01 );
    ppgWriteReg( REG_FIFO_CNTRL, 0x00 );
    ppgWriteReg( REG_SEQ_START,  SEQ_START_BIT );

    // Drain periodically to prevent overflow, then do a final drain after stopping
    uint32_t elapsed = 0;
    while( elapsed < durationMs )
    {
        uint32_t sleepMs = durationMs - elapsed;
        if( sleepMs > PPG_DRAIN_INTERVAL_MS ) sleepMs = PPG_DRAIN_INTERVAL_MS;
        delay( sleepMs );
        elapsed += sleepMs;
        ppgDrainFifoInto( buffer, count, maxCount );
    }

    ppgWriteReg( REG_SEQ_START, 0x00 );
    ppgDrainFifoInto( buffer, count, maxCount );
}

// Computes the mean of an array of uint32 values.
static float ppgArrayMean( const uint32_t* data, uint16_t length )
{
    if( length == 0 ) return 0.0f;
    float sum = 0.0f;
    for( uint16_t i = 0; i < length; i++ )
        sum += ( float )data[ i ];
    return sum / ( float )length;
}

// Applies a DC-blocking high-pass filter (alpha ~ 0.95, ~1.6 Hz cutoff at 100 Hz).
static void ppgHighPassFilter( const uint32_t* input, float* output, uint16_t length )
{
    if( length == 0 ) return;
    float prevInput  = ( float )input[ 0 ];
    float prevOutput = 0.0f;
    const float alpha = 0.95f;
    for( uint16_t i = 0; i < length; i++ )
    {
        float x     = ( float )input[ i ];
        float y     = alpha * ( prevOutput + x - prevInput );
        prevInput   = x;
        prevOutput  = y;
        output[ i ] = y;
    }
}

// Finds local maxima above a threshold with minimum spacing; returns peak count.
// Writes up to maxPeaks peak indices into peakIndices.
static uint16_t ppgDetectPeaks( const float* signal, uint16_t length,
                                 uint16_t* peakIndices, uint16_t maxPeaks,
                                 float minAmplitude, uint16_t minSpacingSamples )
{
    uint16_t peakCount   = 0;
    uint16_t lastPeakIdx = 0;

    for( uint16_t i = 1; i < length - 1 && peakCount < maxPeaks; i++ )
    {
        bool isLocalMax       = ( signal[ i ] > signal[ i - 1 ] ) && ( signal[ i ] > signal[ i + 1 ] );
        bool aboveThreshold   = signal[ i ] > minAmplitude;
        bool sufficientSpacing = ( peakCount == 0 ) || ( ( i - lastPeakIdx ) >= minSpacingSamples );

        if( isLocalMax && aboveThreshold && sufficientSpacing )
        {
            peakIndices[ peakCount++ ] = i;
            lastPeakIdx = i;
        }
    }
    return peakCount;
}

// Computes RMSSD (root mean square of successive RR-interval differences) in ms.
static uint16_t ppgComputeRmssd( const uint16_t* rrIntervals, uint16_t count )
{
    if( count < 2 ) return 0;
    float sumSqDiff  = 0.0f;
    uint16_t diffCount = 0;
    for( uint16_t i = 1; i < count; i++ )
    {
        float diff = ( float )rrIntervals[ i ] - ( float )rrIntervals[ i - 1 ];
        sumSqDiff += diff * diff;
        diffCount++;
    }
    if( diffCount == 0 ) return 0;
    return ( uint16_t )sqrtf( sumSqDiff / ( float )diffCount );
}

// Detects R-peaks in the green PPG channel and computes heart rate and HRV (RMSSD).
// Green light (~520 nm) has stronger absorption contrast for PPG and is preferred for HR/HRV.
static void ppgComputeHrAndHrv()
{
    const uint16_t sampleCount = ppgSampleCount < PPG_BUFFER_SIZE ? ppgSampleCount : PPG_BUFFER_SIZE;
    if( sampleCount < 50 )
    {
        DBG( "PPG", "HR/HRV: too few samples (%u < 50), skipping", sampleCount );
        ppgHeartRate = 0;
        ppgHrv       = 0;
        return;
    }

    static float acSignal[ PPG_BUFFER_SIZE ];
    ppgHighPassFilter( ppgGreenBuffer, acSignal, sampleCount );

    // Determine adaptive peak threshold (25% of peak-to-peak amplitude)
    float sigMax = acSignal[ 0 ], sigMin = acSignal[ 0 ];
    for( uint16_t i = 1; i < sampleCount; i++ )
    {
        if( acSignal[ i ] > sigMax ) sigMax = acSignal[ i ];
        if( acSignal[ i ] < sigMin ) sigMin = acSignal[ i ];
    }
    float threshold = sigMin + 0.25f * ( sigMax - sigMin );

    // Minimum spacing: ~33 samples at 100 Hz = 330 ms → ~182 BPM max
    static uint16_t peakIndices[ 64 ];
    uint16_t peakCount = ppgDetectPeaks( acSignal, sampleCount, peakIndices, 64, threshold, 33 );

    DBG( "PPG", "HR/HRV: %u peaks detected (threshold=%.1f)", peakCount, threshold );

    if( peakCount < 2 )
    {
        DBG( "PPG", "HR/HRV: insufficient peaks (<2), cannot compute" );
        ppgHeartRate = 0;
        ppgHrv       = 0;
        return;
    }

    // Compute RR intervals (in ms, assuming 100 Hz sample rate)
    static uint16_t rrIntervals[ 63 ];
    const float msPerSample = 10.0f;  // 1000ms / 100Hz
    for( uint16_t i = 1; i < peakCount; i++ )
        rrIntervals[ i - 1 ] = ( uint16_t )( ( peakIndices[ i ] - peakIndices[ i - 1 ] ) * msPerSample );
    uint16_t rrCount = peakCount - 1;

    // Mean heart rate from mean RR interval
    float sumRr = 0.0f;
    for( uint16_t i = 0; i < rrCount; i++ )
        sumRr += ( float )rrIntervals[ i ];
    float meanRr = sumRr / ( float )rrCount;
    if( meanRr > 0.0f )
        ppgHeartRate = ( uint16_t )( 60000.0f / meanRr );
    else
        ppgHeartRate = 0;

    // Clamp to physiological range
    if( ppgHeartRate < 30 || ppgHeartRate > 220 )
        ppgHeartRate = 0;

    ppgHrv = ppgComputeRmssd( rrIntervals, rrCount );
}

// Computes SpO2 from the ratio of red/IR AC and DC components using an empirical formula.
static void ppgComputeSpO2()
{
    const uint16_t sampleCount = ppgSampleCount < PPG_BUFFER_SIZE ? ppgSampleCount : PPG_BUFFER_SIZE;
    if( sampleCount < 50 )
    {
        DBG( "PPG", "SpO2: too few samples (%u < 50), skipping", sampleCount );
        ppgSpO2 = 0;
        return;
    }

    // DC component: mean of each channel
    float dcRed = ppgArrayMean( ppgRedBuffer, sampleCount );
    float dcIr  = ppgArrayMean( ppgIrBuffer, sampleCount );

    if( dcRed < 1.0f || dcIr < 1.0f )
    {
        ppgSpO2 = 0;
        return;
    }

    // AC component: RMS of DC-removed signal
    float sumSqRed = 0.0f, sumSqIr = 0.0f;
    for( uint16_t i = 0; i < sampleCount; i++ )
    {
        float diffRed = ( float )ppgRedBuffer[ i ] - dcRed;
        float diffIr  = ( float )ppgIrBuffer[ i ]  - dcIr;
        sumSqRed += diffRed * diffRed;
        sumSqIr  += diffIr  * diffIr;
    }
    float acRed = sqrtf( sumSqRed / ( float )sampleCount );
    float acIr  = sqrtf( sumSqIr  / ( float )sampleCount );

    if( acIr < 1.0f || dcIr < 1.0f )
    {
        ppgSpO2 = 0;
        return;
    }

    // R ratio: (AC_red / DC_red) / (AC_ir / DC_ir)
    float R = ( acRed / dcRed ) / ( acIr / dcIr );

    // Empirical formula: SpO2 = 104 - 17*R  (placeholder, calibrate with reference)
    float spo2f = 104.0f - 17.0f * R;
    DBG( "PPG", "SpO2: R=%.3f dcRed=%.1f dcIr=%.1f acRed=%.1f acIr=%.1f => SpO2=%.1f",
        R, dcRed, dcIr, acRed, acIr, spo2f );

    if( spo2f < 70.0f )  spo2f = 70.0f;
    if( spo2f > 100.0f ) spo2f = 100.0f;
    ppgSpO2 = ( uint8_t )spo2f;
}

// Initialises the AS7038RB: enables LDO/oscillator, configures LEDs, PDs, OFE, ADC, and sequencer.
// Writes all peripheral register configuration (LEDs, PDs, OFE, ADC, sequencer).
// Called from ppgInit() on first boot and from ppgPowerUp() after each LDO-off cycle,
// because the chip's volatile registers reset when the internal LDO is off.
static void ppgConfigure()
{
    // LED_CFG: enable all three LEDs and signal reference.
    // ppgRunPass() selects which LED actually fires each pass by writing LED12_MODE/LED34_MODE;
    // this register just enables the driver sources so current can flow when the sequencer fires.
    ppgWriteReg( REG_LED_CFG, LED_CFG_SIGREF_EN | LED_CFG_LED1_EN | LED_CFG_LED2_EN | LED_CFG_LED3_EN );

    // Set all LED currents to ~10 mA.
    // Current registers are non-volatile across LED_CFG/MODE changes; must be written once here.
    // 10-bit Curr: CURRH holds Curr[9:2], CURRL holds Curr[1:0] in D7:D6
    ppgWriteReg( REG_LED1_CURRH, ( uint8_t )( LED_CURRENT_10MA >> 2 ) );
    ppgWriteReg( REG_LED1_CURRL, ( uint8_t )( ( LED_CURRENT_10MA & 0x03 ) << 6 ) );
    ppgWriteReg( REG_LED2_CURRH, ( uint8_t )( LED_CURRENT_10MA >> 2 ) );
    ppgWriteReg( REG_LED2_CURRL, ( uint8_t )( ( LED_CURRENT_10MA & 0x03 ) << 6 ) );
    ppgWriteReg( REG_LED3_CURRH, ( uint8_t )( LED_CURRENT_10MA >> 2 ) );
    ppgWriteReg( REG_LED3_CURRL, ( uint8_t )( ( LED_CURRENT_10MA & 0x03 ) << 6 ) );

    // Enable photodiodes PD1 and PD2
    ppgWriteReg( REG_PD_CFG, PD_CFG_PD1 | PD_CFG_PD2 );

    // Enable transimpedance amplifier
    ppgWriteReg( REG_PD_AMPCFG, PD_AMP_EN );

    // Enable OFE1 and OFE bias; OFE2 kept enabled as a fallback (harmless when only OFE1 is sampled)
    ppgWriteReg( REG_OFE_CFGA, OFE_CFG_OFE2_EN | OFE_CFG_OFE1_EN | OFE_CFG_BIAS_EN );

    // ADC captures OFE1 only — one entry per sequence, corresponding to LED1 (red)
    ppgWriteReg( REG_ADC_CHANNEL_MASK_L, ADC_MASK_OFE1 );

    // Enable ADC
    ppgWriteReg( REG_ADC_CFGB, ADC_CFGB_ADC_EN );

    // Default sequencer mode: LED1 (red) fires every sequence; LED2/3 off.
    // ppgRunPass() overrides these registers for each sampling pass.
    ppgWriteReg( REG_LED12_MODE, LED1_MODE_SEQUENCER ); // led1_mode=010, led2_mode=000
    ppgWriteReg( REG_LED34_MODE, 0x00 );                // led3_mode=000, led4_mode=000

    // Enable the sequencer (seq_en bit 0 defaults to 0; must be set before SEQ_START has effect)
    ppgWriteReg( REG_MAN_SEQ_CFG, MAN_SEQ_CFG_SEQ_EN );

    // Sequencer timing: continuous run, ~100 Hz.
    // Period = 78 ticks (SEQ_PER=0x4E); LED on from tick 2 to 64 (62-tick window);
    // ADC fires at tick 72 — after LED_STO(64) and before period end(78).
    ppgWriteReg( REG_SEQ_CNT,     0x00 ); // infinite sequences
    ppgWriteReg( REG_SEQ_DIV,     0xFF ); // maximum divider (~7.8 kHz tick from 2 MHz oscillator)
    ppgWriteReg( REG_SEQ_PER,     0x4E ); // period = 78 ticks → ~100 Hz
    ppgWriteReg( REG_SEQ_LED_STA, 0x02 ); // LED on at tick 2
    ppgWriteReg( REG_SEQ_LED_STO, 0x40 ); // LED off at tick 64
    ppgWriteReg( REG_SEQ_ADC,     0x48 ); // ADC at tick 72 (after LED_STO=64, before PER=78)

    DBG( "PPG", "Registers configured (LED1 red, OFE1, ADC, sequencer enabled)" );
}

void ppgInit( TwoWire& wire )
{
    ppgI2cBus      = &wire;
    ppgWriteIndex  = 0;
    ppgSampleCount = 0;
    ppgResultsValid = false;

    DBG( "PPG", "Init: enabling LDO + oscillator" );

    // Enable internal LDO and oscillator; datasheet Ton = 35 ms
    ppgWriteReg( REG_CONTROL, CTRL_LDO_EN | CTRL_OSC_EN );
    delay( 40 );

    // Verify chip ID
    uint8_t chipId = ppgReadReg( REG_ID );
    DBG( "PPG", "Chip ID=0x%02X (expected 0x44)", chipId );

    ppgConfigure();
    DBG( "PPG", "Init complete - sequencer configured for ~100 Hz" );
}

// Resets sample buffers, clears the FIFO, and starts the sequencer.
void ppgStartSampling()
{
    ppgWriteIndex  = 0;
    ppgSampleCount = 0;
    ppgResultsValid = false;

    // Clear FIFO before starting
    ppgWriteReg( REG_FIFO_CNTRL, 0x01 );  // fifo_clear
    ppgWriteReg( REG_FIFO_CNTRL, 0x00 );

    // Start sequencer
    ppgWriteReg( REG_SEQ_START, SEQ_START_BIT );
    DBG( "PPG", "Sequencer started, FIFO cleared" );
}

// Stops the sequencer and drains any remaining FIFO entries into the red buffer.
// Valid for single-pass use (ppgStartSampling + ppgStopSampling); for full 3-channel
// collection use ppgCollectSamples() instead.
void ppgStopSampling()
{
    ppgWriteReg( REG_SEQ_START, 0x00 );
    DBG( "PPG", "Sequencer stopped, draining remaining FIFO" );

    uint8_t fifoStatus = ppgReadReg( REG_FIFOSTATUS );
    if( fifoStatus & 0x01 )
        DBG( "PPG", "WARNING: FIFO overflow - samples were lost" );

    ppgDrainFifoInto( ppgRedBuffer, &ppgWriteIndex, PPG_BUFFER_SIZE );
    ppgSampleCount = ppgWriteIndex;
    DBG( "PPG", "FIFO drain complete: %u samples", ppgSampleCount );

    ppgComputeHrAndHrv();
    ppgComputeSpO2();
    ppgResultsValid = ( ppgHeartRate > 0 );

    DBG( "PPG", "Algorithm results: HR=%u SpO2=%u HRV=%u valid=%d",
        ppgHeartRate, ppgSpO2, ppgHrv, ppgResultsValid );
}

// Runs three sequential LED passes (green → red → IR), each of durationMs, with periodic
// mid-window FIFO draining. Fills all three sample buffers, then runs all algorithms.
void ppgCollectSamples( uint32_t durationMs )
{
    ppgWriteIndex   = 0;
    ppgSampleCount  = 0;
    ppgResultsValid = false;

    uint16_t greenCount = 0, redCount = 0, irCount = 0;

    // Pass 1: green (LED3) — primary channel for HR and HRV
    DBG( "PPG", "Pass 1/3: green (LED3), %lu ms", ( unsigned long )durationMs );
    ppgRunPass( LED_CFG_LED3_EN, 0x00, LED3_MODE_SEQUENCER,
                ppgGreenBuffer, &greenCount, PPG_BUFFER_SIZE, durationMs );
    DBG( "PPG", "Pass 1 complete: %u green samples", greenCount );

    // Pass 2: red (LED1) — SpO2 numerator channel
    DBG( "PPG", "Pass 2/3: red (LED1), %lu ms", ( unsigned long )durationMs );
    ppgRunPass( LED_CFG_LED1_EN, LED1_MODE_SEQUENCER, 0x00,
                ppgRedBuffer, &redCount, PPG_BUFFER_SIZE, durationMs );
    DBG( "PPG", "Pass 2 complete: %u red samples", redCount );

    // Pass 3: IR (LED2) — SpO2 denominator channel
    DBG( "PPG", "Pass 3/3: IR (LED2), %lu ms", ( unsigned long )durationMs );
    ppgRunPass( LED_CFG_LED2_EN, LED2_MODE_SEQUENCER, 0x00,
                ppgIrBuffer, &irCount, PPG_BUFFER_SIZE, durationMs );
    DBG( "PPG", "Pass 3 complete: %u IR samples", irCount );

    // Use the minimum sample count across channels so no algorithm reads past valid data
    ppgSampleCount = greenCount < redCount ? greenCount : redCount;
    if( irCount < ppgSampleCount ) ppgSampleCount = irCount;
    DBG( "PPG", "Collection complete: %u valid samples (green=%u red=%u ir=%u)",
        ppgSampleCount, greenCount, redCount, irCount );

    ppgComputeHrAndHrv();
    ppgComputeSpO2();
    ppgResultsValid = ( ppgHeartRate > 0 );

    DBG( "PPG", "Algorithm results: HR=%u SpO2=%u HRV=%u valid=%d",
        ppgHeartRate, ppgSpO2, ppgHrv, ppgResultsValid );
}

// Writes the last computed heart rate in BPM to bpm. Returns false if results are invalid.
bool ppgGetHeartRate( uint16_t* bpm )
{
    if( !ppgResultsValid || bpm == nullptr ) return false;
    *bpm = ppgHeartRate;
    return true;
}

// Writes the last computed SpO2 percentage to spo2. Returns false if results are invalid.
bool ppgGetSpO2( uint8_t* spo2 )
{
    if( !ppgResultsValid || spo2 == nullptr ) return false;
    *spo2 = ppgSpO2;
    return true;
}

// Writes the last computed HRV (RMSSD ms) to rmssd. Returns false if results are invalid.
bool ppgGetHrv( uint16_t* rmssd )
{
    if( !ppgResultsValid || rmssd == nullptr ) return false;
    *rmssd = ppgHrv;
    return true;
}

// Drains all available FIFO entries and returns the most recent sample in all three channels.
// Single-LED red mode: all three output pointers receive the same red value.
// Returns false if no FIFO entries are available.
bool ppgReadLatestFifoSample( uint16_t* red, uint16_t* ir, uint16_t* green, uint8_t* fifoLevel )
{
    uint8_t level = ppgReadReg( REG_FIFOLEVEL );
    if( fifoLevel ) *fifoLevel = level;

    if( level < 1 ) return false;

    uint16_t lastSample = 0;

    // Drain all entries; keep only the last (most recent)
    for( uint8_t i = 0; i < level; i++ )
    {
        uint8_t lo = ppgReadReg( REG_FIFOL );
        uint8_t hi = ppgReadReg( REG_FIFOH ) & 0x3F;
        lastSample = ( ( uint16_t )hi << 8 ) | lo;
    }

    // Single-LED red mode: all channels report the same red OFE1 value
    if( red )   *red   = lastSample;
    if( ir )    *ir    = lastSample;
    if( green ) *green = lastSample;
    return true;
}

// Clears the CONTROL register, putting the chip into low-power state (~0.5 µA).
void ppgPowerDown()
{
    DBG( "PPG", "Powering down (LDO+OSC off)" );
    ppgWriteReg( REG_CONTROL, 0x00 );
}

// Re-enables the LDO and oscillator, waits for stabilisation, then reloads all
// peripheral registers. Leaves the sensor ready for ppgStartSampling().
void ppgPowerUp()
{
    DBG( "PPG", "Powering up (LDO+OSC on, waiting 40 ms)" );
    ppgWriteReg( REG_CONTROL, CTRL_LDO_EN | CTRL_OSC_EN );
    delay( 40 );  // datasheet Ton = 35 ms
    ppgConfigure();
}
