# AGENTS.md

## Project Overview

HALSER attitude sensor firmware — an ESP32-C3 firmware that reads an ICM-20948 9-DoF IMU over I2C, computes calibrated vessel roll, pitch, and rate of turn, and publishes them to NMEA 2000 and Signal K via the HALSER board.

## Build Commands

```bash
# Build firmware (default_envs = halser_espidf)
pio run

# Arduino compile check
pio run -e halser

# Upload to connected board
pio run -t upload

# Run unit tests (native platform)
pio test -e native
```

## Architecture

### Data Flow

```
ICM-20948 (I2C, DMP game rotation vector, 10 Hz)
  → AttitudeSensor (quaternion → calibrated roll/pitch + rates)
    → N2kAttitudeSender (PGN 127257, 100 ms)
        → CountingNMEA2000 (TWAI, GPIO 4 TX / GPIO 5 RX)
    → N2kRateOfTurnSender (PGN 127251, 100 ms)
        → CountingNMEA2000
    → Signal K output (via WiFi/WebSocket)

Web UI ←→ CalibrationConfig ←→ AttitudeCalibration (bow + level reference)
```

### Source Layout

**Attitude Math** (`src/`):
- `attitude_calibration.h` — Pure math: gravity-from-quaternion, bow projection, boat-frame derivation, 24-orientation calibration. No Arduino or SensESP dependencies; host-tested
- `rate_filter.h` — Exponential smoother that reseeds from a fresh sample when the previous value is non-finite; host-tested
- `imu.h` — `AttitudeSensor`: ICM-20948 DMP initialization with retry, quaternion read, rate differentiation. Publishes roll, pitch, yaw_rate, roll_rate, pitch_rate as `ObservableValue<float>`
- `calibration_config.h` — `CalibrationConfig`: web UI calibration card (bow dropdown + level capture on Save), persistence to flash

**NMEA 2000 Output** (`src/sender/`):
- `n2k_senders.h` — `N2kAttitudeSender` (PGN 127257) and `N2kRateOfTurnSender` (PGN 127251). Both take `CountingNMEA2000*` and use `RepeatExpiring` (5 s timeout) with NaN→N2kDoubleNA guard

**Application** (`src/`):
- `main.cpp` — Entry point; wires IMU → N2K + Signal K, calibration card, diagnostics. OTA password is a placeholder (`change-me`)
- `counting_nmea2000.h` — `tNMEA2000_esp32` subclass that counts accepted `SendMsg` calls; senders take `CountingNMEA2000*` (not `tNMEA2000*`) because `SendMsg` is not virtual

### Hardware Pin Assignments

| Pin | Function |
|-----|----------|
| GPIO 4 | CAN TX |
| GPIO 5 | CAN RX |
| GPIO 6 | I2C SDA |
| GPIO 7 | I2C SCL |
| GPIO 8 | RGB LED (SK6805) |
| GPIO 9 | Button |

### NMEA 2000 PGNs

| PGN | Description | Interval |
|-----|-------------|----------|
| 127257 | Vessel Attitude (roll, pitch; yaw = N/A) | 100 ms |
| 127251 | Rate of Turn | 100 ms |

Default source address: 75.

### Signal K Paths

- `navigation.attitude` — vessel attitude (roll, pitch in radians; yaw null)
- `navigation.rateOfTurn` — yaw rate in rad/s (positive = starboard)

## Dependencies

- SensESP ^3.5.0 — IoT framework (WiFi, web UI, Signal K)
- SparkFun ICM-20948 ^1.2.13 — IMU driver with DMP support
- NMEA2000-library ^4.17.2 — NMEA 2000 message handling
- NMEA2000_twai — ESP32 TWAI (CAN) driver
- elapsedMillis ^1.0.6 — Timing utilities
- esp_websocket_client 1.7.0 — WebSocket support (Espressif component, per-env sourcing)
