# ESP32 Biometric Smartwatch

Firmware for Team 314's Senior Design Project: a biometric smartwatch built on the ESP32-S3, developed with the Arduino framework via PlatformIO.

The watch measures heart rate, SpO2, HRV, and step count, transmits data to an Android companion app over BLE, and renders a live metrics UI on a built-in TFT display. The project is mostly complete, but it may receive more updates in the future.

## Hardware

**Board:** Adafruit ESP32-S3 Feather ReverseTFT

**Sensors (I2C):**
- AS7038RB: optical biometric sensor (PPG); provides heart rate, SpO2, and HRV via green, red, and IR LEDs
- LSM6DSO: 6-axis IMU; provides step counting and lift-to-wake via the embedded pedometer and tilt detection algorithms
- MAX17048 (built into the Feather board): battery fuel gauge; reports state of charge and charging status

**Display:** 240x135 ST7789 TFT (built into the Feather board), driven via Adafruit ST7789 + GFX libraries

## Architecture

`src/main.cpp` is the Arduino entry point and top-level orchestrator. All component logic lives in dedicated modules under `src/` and `include/`:

| Module | Responsibility |
|---|---|
| `as7038rb` | PPG sensor driver (HR, SpO2, HRV) |
| `lsm6dso` | IMU driver (step count, motion artifact rejection) |
| `display` | TFT UI rendering |
| `bluetooth` | BLE peripheral, advertising, data transmission, buffered sync |
| `dataBuffer` | Local ring buffer for readings when BLE is disconnected |
| `powerManager` | Sleep/wake logic, peripheral enable/disable |
| `battery` | MAX17048 fuel gauge driver |

The firmware uses FreeRTOS tasks pinned to separate cores: sensor sampling runs on one core, BLE transmission and display management on the other.

## Sampling & Power Strategy

The MCU runs a fixed 15s sampling window followed by a 45s idle period (one reading per minute at rest). During idle the MCU sleeps and the PPG LEDs are off, keeping average LED current low (~30 mA peak at 10 mA per LED, ~25% duty cycle). The CPU runs at 80 MHz (reduced from 240 MHz) to save power while maintaining BLE.

## BLE & Android Integration

The watch advertises as a BLE peripheral and connects to a companion Android app. Readings are transmitted as 19-byte packed payloads carrying a Unix epoch millisecond timestamp, HR, SpO2, HRV, step count, and validity flag. Readings are buffered locally when no connection is active and flushed automatically on reconnect. The watch requests a time sync from Android ~3s after connect and re-syncs periodically; sync state persists across deep sleep.

## Build & Flash

```bash
# Build
pio run

# Build and upload
pio run --target upload

# Upload and open serial monitor
pio run --target upload && pio device monitor
```

## Testing Modes

The `TESTING_MODE` constant in `src/main.cpp` controls which hardware is initialized and what data is used:

| Mode | Description |
|---|---|
| 0 | Normal operation: all sensors, BLE, power management |
| 1 | Dummy data: no sensor init; drifting random values for UI/BLE testing |
| 2 | PPG raw test: AS7038RB only; prints raw FIFO counts to serial |
| 3 | IMU raw test: LSM6DSO only; prints accel/gyro/steps to serial |
| 4 | Real PPG + dummy IMU |
| 5 | Real IMU + dummy PPG |

## Dependencies

Declared in `platformio.ini`. Key libraries:
- `adafruit/Adafruit ST7789` and `adafruit/Adafruit GFX Library`
- `stm32duino/LSM6DSO`
- `h2zero/NimBLE-Arduino` v2.5.0
