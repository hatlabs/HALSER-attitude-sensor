# HALSER Attitude Sensor

ESP32-C3 firmware that reads an ICM-20948 9-DoF IMU over I2C, computes calibrated vessel roll, pitch, and rate of turn, and publishes them to NMEA 2000 and Signal K via the [HALSER](https://hatlabs.fi/halser/) board.

## Wiring

The ICM-20948 breakout connects to the HALSER I2C header:

| ICM-20948 | HALSER I2C header |
|-----------|-------------------|
| VIN / VCC | 3.3 V |
| GND       | GND   |
| SDA       | SDA (GPIO 6) |
| SCL       | SCL (GPIO 7) |

**AD0 strap:** the firmware tries address 0x69 first (AD0 high, the SparkFun default), then 0x68. If the breakout pulls AD0 low, set `kIMUAd0 = 0` in `src/main.cpp`.

## Building and flashing

```bash
# Build the flashable environment (first build downloads ESP-IDF, several minutes)
pio run

# Flash via USB
pio run -t upload

# Run the native test suites
pio test -e native
```

Always flash the `halser_espidf` environment (the default). The plain `halser` environment builds faster but lacks the dynamic TLS buffer — against a TLS-enabled Signal K server the device boots, joins WiFi, but Signal K stays Disconnected (`-0x7F00` in the log).

**OTA password:** the firmware ships with the placeholder `change-me`. Change it in `src/main.cpp` before deploying.

**Arduino to espidf USB note:** if the device was last flashed with an arduino-only build, the first espidf flash may need a manual boot-mode entry (hold BOOT, press RESET, release BOOT) because the USB descriptor changes.

## Calibration

The web UI (http://attitude.local after connecting to the device's WiFi) has a single **Attitude Calibration** card with two controls:

1. **Bow Direction** — which chip axis points toward the bow. The six signed axes (+X, -X, +Y, -Y, +Z, -Z) cover flat, on-edge, and bulkhead mounts. Pick the closest axis; the exact mounting tilt is absorbed by the level capture.

2. **Save** — with the boat at rest on an even keel, saving the card captures the current pose as the level reference (zero heel and trim). There is no separate "Set Level" button; Save is the trigger.

### Procedure

1. Mount the HALSER + ICM-20948 in the boat.
2. Open the web UI and choose the bow direction from the dropdown.
3. With the boat level and at rest, press **Save**.
4. The status page shows "Calibrated" and reports roll and pitch in degrees.

### Orientation table

The 24 mounting orientations reduce to 6 bow-axis choices. The level capture absorbs the remaining 4 rotations per axis (the 4 possible "which way is up" at that bow). Example mappings:

| Mount | Bow axis |
|-------|----------|
| Board flat, USB connector toward bow | +X (check your breakout's axis markings) |
| Board flat, USB connector toward stern | -X |
| Board on its long edge, short edge toward bow | +Y or -Y |
| Board on a bulkhead, face toward bow | +Z or -Z |

### What can go wrong

- **Bow axis near vertical:** if the chosen axis points up or down at the current pose, the firmware rejects the level capture and shows a guiding status. Pick the axis that faces the bow, not the one that faces up.
- **Saving while heeled:** Save re-captures the level reference every time a bow is set. Saving while the boat is heeled shifts the zero point; re-save on an even keel.
- **IMU still settling:** the DMP needs about 40 seconds after boot to converge. Saving before that shows "IMU still settling — wait ~40 s after boot, then Save again".

## Sign conventions

| Measurement | Positive direction | Unit | Signal K path |
|-------------|-------------------|------|---------------|
| Roll (heel) | Starboard heel | rad | `navigation.attitude` (roll field) |
| Pitch (trim) | Bow up | rad | `navigation.attitude` (pitch field) |
| Rate of turn | Starboard (clockwise from above) | rad/s | `navigation.rateOfTurn` |

Yaw is always null (no magnetic heading reference). Roll rate and pitch rate are computed internally but have no standard Signal K path; they are commented out in `src/main.cpp`.

## NMEA 2000

| PGN | Description | Interval |
|-----|-------------|----------|
| 127257 | Vessel Attitude (roll, pitch; yaw = N/A) | 100 ms |
| 127251 | Rate of Turn | 100 ms |

Default source address: 75.

## Dependencies

- [SensESP](https://github.com/SignalK/SensESP) ^3.5.0
- [SparkFun ICM-20948](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary) ^1.2.13
- [NMEA2000-library](https://github.com/ttlappalainen/NMEA2000) ^4.17.2
- [NMEA2000_twai](https://github.com/skarlsson/NMEA2000_twai)

## License

Apache 2.0 — Copyright (c) Hat Labs Oy
