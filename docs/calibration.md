# Attitude Calibration

## Model

The ICM-20948 DMP produces a 6-axis game rotation vector (accelerometer + gyroscope, no magnetometer). This quaternion fuses into a drift-stable orientation whose yaw is relative (arbitrary origin each boot) but whose tilt (gravity direction) is absolute.

Roll and pitch are derived from the gravity vector alone, which makes them reboot-stable: the gravity direction at the level reference pose is stored once and reused across power cycles, regardless of the DMP's yaw origin.

## Stored quantities

Two chip-frame quantities define the boat frame:

1. **Bow axis** — which signed chip axis points toward the bow. User-selected from a dropdown of 6 options (+X, -X, +Y, -Y, +Z, -Z). Persisted as a label string.

2. **Level reference** — the gravity (down) unit vector in the chip body frame, captured when the user presses Save with the boat at rest on an even keel. Persisted as three floats (gx, gy, gz).

## Frame derivation

From the bow axis and the level reference:

```
down    = normalize(g_level)
forward = normalize(bow − (bow·down) down)
stbd    = down × forward
```

The bow axis is projected onto the plane perpendicular to `down`, so the exact mounting tilt is absorbed. The result is a right-handed boat frame: x = forward (bow), y = starboard, z = down.

## Calibrated output

```
g_current = gravity_from_quat(w, x, y, z)
g_boat    = (dot(g_current, forward), dot(g_current, stbd), dot(g_current, down))
roll      = atan2(g_boat.y, g_boat.z)
pitch     = atan2(-g_boat.x, sqrt(g_boat.y² + g_boat.z²))
```

Sign convention: roll positive = heel to starboard; pitch positive = bow up.

## Guards

- **Bow near vertical:** if the bow axis lies within ~8° of vertical at capture (horizontal projection < 0.139), the level capture is rejected. The user must pick the axis that faces the bow, not up/down.
- **Uncalibrated passthrough:** until both a bow is set and a level reference is captured, `compute()` returns the raw chip-frame roll/pitch (identical to an uncalibrated device).
- **Attitude gating:** roll and pitch are published as NaN (Signal K null, N2K not-available) until calibrated, leveled, and the DMP has settled (~40 s after boot). This prevents the bus from carrying angles derived from the raw chip frame or an unconverged DMP.
