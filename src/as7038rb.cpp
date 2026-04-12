#include "as7038rb.h"
#include "debug.h"
#include <Arduino.h>
#include <math.h>

// AS7038RB I2C address (fixed, no address-select pin)
static const uint8_t AS7038RB_I2C_ADDR = 0x60;

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
static const uint8_t REG_ADC_CHANNEL_MASK_L   = 0x8B; // tia D0, ofe1 D1, ofe2 D2, sd1 D3, sd2 D4, temp D5, afe D6, pregain D7
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
static const uint8_t ADC_MASK_OFE2 = 0x04; // D2 - capture OFE2

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

    // Enable LED1 (red), LED2 (IR), and LED3 (green), plus signal reference
    ppgWriteReg( REG_LED_CFG, LED_CFG_SIGREF_EN | LED_CFG_LED3_EN | LED_CFG_LED2_EN | LED_CFG_LED1_EN );

    // Set LED1 (red) current to ~10 mA
    // 10-bit Curr: CURRH holds Curr[9:2], CURRL holds Curr[1:0] in D7:D6
    ppgWriteReg( REG_LED1_CURRH, ( uint8_t )( LED_CURRENT_10MA >> 2 ) );
    ppgWriteReg( REG_LED1_CURRL, ( uint8_t )( ( LED_CURRENT_10MA & 0x03 ) << 6 ) );

    // Set LED2 (IR) to the same current
    ppgWriteReg( REG_LED2_CURRH, ( uint8_t )( LED_CURRENT_10MA >> 2 ) );
    ppgWriteReg( REG_LED2_CURRL, ( uint8_t )( ( LED_CURRENT_10MA & 0x03 ) << 6 ) );

    // Set LED3 (green) to the same current
    ppgWriteReg( REG_LED3_CURRH, ( uint8_t )( LED_CURRENT_10MA >> 2 ) );
    ppgWriteReg( REG_LED3_CURRL, ( uint8_t )( ( LED_CURRENT_10MA & 0x03 ) << 6 ) );

    // Enable photodiodes PD1 and PD2
    ppgWriteReg( REG_PD_CFG, PD_CFG_PD1 | PD_CFG_PD2 );

    // Enable transimpedance amplifier
    ppgWriteReg( REG_PD_AMPCFG, PD_AMP_EN );

    // Enable OFE1 (red), OFE2 (IR), and OFE bias
    ppgWriteReg( REG_OFE_CFGA, OFE_CFG_OFE2_EN | OFE_CFG_OFE1_EN | OFE_CFG_BIAS_EN );

    // Select OFE1 and OFE2 as ADC capture channels
    ppgWriteReg( REG_ADC_CHANNEL_MASK_L, ADC_MASK_OFE1 | ADC_MASK_OFE2 );

    // Enable ADC
    ppgWriteReg( REG_ADC_CFGB, ADC_CFGB_ADC_EN );

    // Sequencer: continuous run (SEQ_CNT = 0 → infinite), default clock divider and period.
    // These values produce roughly 100 Hz PPG sample rate; tune via SEQ_DIV and SEQ_PER
    // if a different rate is needed.
    ppgWriteReg( REG_SEQ_CNT, 0x00 );     // infinite sequences
    ppgWriteReg( REG_SEQ_DIV, 0xFF );     // maximum divider (~7.8 kHz tick from 2 MHz oscillator)
    ppgWriteReg( REG_SEQ_PER, 0x4E );     // ~78 ticks per period → ~100 Hz
    ppgWriteReg( REG_SEQ_LED_STA, 0x02 );
    ppgWriteReg( REG_SEQ_LED_STO, 0x40 );
    ppgWriteReg( REG_SEQ_ADC,     0x60 );

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

// Stops the sequencer, drains the FIFO into sample buffers, and runs HR/SpO2/HRV algorithms.
void ppgStopSampling()
{
    // Stop sequencer
    ppgWriteReg( REG_SEQ_START, 0x00 );
    DBG( "PPG", "Sequencer stopped, draining FIFO" );

    // Check for overflow before draining
    uint8_t fifoStatus = ppgReadReg( REG_FIFOSTATUS );
    if( fifoStatus & 0x01 )
        DBG( "PPG", "WARNING: FIFO overflow detected - samples were lost" );

    // Drain the FIFO into PPG buffers.
    // FIFO entries are interleaved: red (slot 0), IR (slot 1), green (slot 2) per sequence cycle.
    // 14-bit samples: lower 8 bits from FIFOL (0xFE), upper 6 bits from FIFOH[5:0] (0xFF).
    uint8_t fifoLevel = ppgReadReg( REG_FIFOLEVEL );
    DBG( "PPG", "FIFO level=%u entries (%u triplets)", fifoLevel, fifoLevel / 3 );
    ppgWriteIndex  = 0;
    ppgSampleCount = 0;

    for( uint8_t i = 0; i < fifoLevel; i++ )
    {
        uint8_t  lo     = ppgReadReg( REG_FIFOL );
        uint8_t  hi     = ppgReadReg( REG_FIFOH ) & 0x3F;
        uint16_t sample = ( ( uint16_t )hi << 8 ) | ( uint16_t )lo;

        uint8_t slot = i % 3;
        if( slot == 0 )  // red
        {
            if( ppgWriteIndex < PPG_BUFFER_SIZE )
                ppgRedBuffer[ ppgWriteIndex ] = ( uint32_t )sample;
        }
        else if( slot == 1 )  // IR
        {
            if( ppgWriteIndex < PPG_BUFFER_SIZE )
                ppgIrBuffer[ ppgWriteIndex ] = ( uint32_t )sample;
        }
        else  // green
        {
            if( ppgWriteIndex < PPG_BUFFER_SIZE )
            {
                ppgGreenBuffer[ ppgWriteIndex ] = ( uint32_t )sample;
                ppgWriteIndex++;
                ppgSampleCount++;
            }
        }
    }

    DBG( "PPG", "FIFO drain complete: %u sample triplets collected", ppgSampleCount );

    // Run algorithms on collected samples
    ppgComputeHrAndHrv();
    ppgComputeSpO2();
    ppgResultsValid = ( ppgHeartRate > 0 );

    DBG( "PPG", "Algorithm results: HR=%u SpO2=%u HRV=%u valid=%d",
        ppgHeartRate, ppgSpO2, ppgHrv, ppgResultsValid );
}

// Convenience wrapper that starts sampling, waits durationMs, then stops.
void ppgCollectSamples( uint32_t durationMs )
{
    ppgStartSampling();
    delay( durationMs );
    ppgStopSampling();
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

// Drains all complete red+IR+green FIFO triplets and returns the most recent sample.
// Returns false if fewer than three FIFO entries are available.
bool ppgReadLatestFifoSample( uint16_t* red, uint16_t* ir, uint16_t* green, uint8_t* fifoLevel )
{
    uint8_t level = ppgReadReg( REG_FIFOLEVEL );
    if( fifoLevel ) *fifoLevel = level;

    // Need at least one complete red+IR+green triplet (three entries)
    if( level < 3 ) return false;

    uint16_t lastRed = 0, lastIr = 0, lastGreen = 0;
    uint8_t  tripletsToRead = level / 3;

    // Drain all complete triplets; keep only the last one (most recent sample)
    for( uint8_t i = 0; i < tripletsToRead; i++ )
    {
        uint8_t lo, hi;
        lo = ppgReadReg( REG_FIFOL );
        hi = ppgReadReg( REG_FIFOH ) & 0x3F;
        lastRed = ( ( uint16_t )hi << 8 ) | lo;

        lo = ppgReadReg( REG_FIFOL );
        hi = ppgReadReg( REG_FIFOH ) & 0x3F;
        lastIr = ( ( uint16_t )hi << 8 ) | lo;

        lo = ppgReadReg( REG_FIFOL );
        hi = ppgReadReg( REG_FIFOH ) & 0x3F;
        lastGreen = ( ( uint16_t )hi << 8 ) | lo;
    }

    if( red )   *red   = lastRed;
    if( ir )    *ir    = lastIr;
    if( green ) *green = lastGreen;
    return true;
}

// Clears the CONTROL register, putting the chip into low-power state (~0.5 µA).
void ppgPowerDown()
{
    DBG( "PPG", "Powering down (LDO+OSC off)" );
    ppgWriteReg( REG_CONTROL, 0x00 );
}

// Re-enables the LDO and oscillator; waits 40 ms for the chip to stabilise.
void ppgPowerUp()
{
    DBG( "PPG", "Powering up (LDO+OSC on, waiting 40 ms)" );
    ppgWriteReg( REG_CONTROL, CTRL_LDO_EN | CTRL_OSC_EN );
    delay( 40 );  // datasheet Ton = 35 ms
}
