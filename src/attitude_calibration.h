#ifndef ATTITUDE_SENSOR_SRC_ATTITUDE_CALIBRATION_H_
#define ATTITUDE_SENSOR_SRC_ATTITUDE_CALIBRATION_H_

#include <cmath>
#include <cstdio>
#include <cstring>

// Pure, hardware-independent attitude calibration math. Includes no Arduino or
// SensESP headers so it compiles host-side for native unit tests.
//
// Pipeline (see docs/calibration.md):
//   DMP quaternion → gravity (down) vector in chip frame → boat frame → roll
//   (heel) / pitch (trim).
//
// The boat frame is fully defined by two chip-frame quantities:
//   - the bow axis: which signed chip axis points toward the bow (user choice);
//   - the level reference "down": the gravity direction captured by Set Level.
// Gravity supplies down directly; the user only has to name the bow, because the
// rotation about the vertical (which horizontal direction is forward) is the one
// thing a gravity vector cannot resolve. From those two:
//   down    = normalize(g_level)
//   forward = normalize(bow − (bow·down) down)   (bow projected onto horizontal)
//   stbd    = down × forward                      (right-handed: y = z × x)
//
// Only the gravity direction feeds roll/pitch, so the result is independent of the
// DMP's arbitrary per-boot yaw origin and therefore reboot-stable. Both stored
// quantities live in the chip frame, so changing the bow selection re-derives the
// frame without re-capturing the level reference.
//
// Boat frame: x = forward (bow), y = starboard, z = down.

namespace attitude_sensor {

struct Vec3 {
  double x, y, z;
};

inline double dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline Vec3 cross(const Vec3& a, const Vec3& b) {
  return Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
              a.x * b.y - a.y * b.x};
}
inline double norm(const Vec3& a) { return std::sqrt(dot(a, a)); }
inline Vec3 normalized(const Vec3& a) {
  double n = norm(a);
  return n > 1e-12 ? Vec3{a.x / n, a.y / n, a.z / n} : a;
}

// Signed chip axis that points toward the bow.
enum class Axis { kPlusX, kMinusX, kPlusY, kMinusY, kPlusZ, kMinusZ };

inline Vec3 axis_vec(Axis a) {
  switch (a) {
    case Axis::kPlusX: return {1, 0, 0};
    case Axis::kMinusX: return {-1, 0, 0};
    case Axis::kPlusY: return {0, 1, 0};
    case Axis::kMinusY: return {0, -1, 0};
    case Axis::kPlusZ: return {0, 0, 1};
    case Axis::kMinusZ: return {0, 0, -1};
  }
  return {1, 0, 0};
}

inline const char* axis_name(Axis a) {
  switch (a) {
    case Axis::kPlusX: return "+X";
    case Axis::kMinusX: return "-X";
    case Axis::kPlusY: return "+Y";
    case Axis::kMinusY: return "-Y";
    case Axis::kPlusZ: return "+Z";
    case Axis::kMinusZ: return "-Z";
  }
  return "?";
}

// Bow-axis catalog: the single source of truth for both the parser and the web-UI
// dropdown. All six signed chip axes are valid — ±X/±Y for flat and on-edge
// mounts, ±Z for bulkhead mounts where a board face points forward. The label is
// generated from the axis, so the dropdown and parser cannot drift.
constexpr char kBowUncalibrated[] = "Uncalibrated";

inline const Axis* bow_axes(unsigned& count) {
  static const Axis kAxes[] = {Axis::kPlusX, Axis::kMinusX, Axis::kPlusY,
                               Axis::kMinusY, Axis::kPlusZ, Axis::kMinusZ};
  count = sizeof(kAxes) / sizeof(kAxes[0]);
  return kAxes;
}

// Render a bow label, e.g. "Bow -Z", into the caller's buffer.
inline void bow_label(Axis bow, char* buf, unsigned len) {
  std::snprintf(buf, len, "Bow %s", axis_name(bow));
}

// Resolve a bow label to a chip axis. Returns false for "Uncalibrated" or any
// unknown label.
inline bool parse_bow(const char* label, Axis& bow) {
  unsigned n;
  const Axis* axes = bow_axes(n);
  char buf[16];
  for (unsigned i = 0; i < n; i++) {
    bow_label(axes[i], buf, sizeof(buf));
    if (std::strcmp(label, buf) == 0) {
      bow = axes[i];
      return true;
    }
  }
  return false;
}

class AttitudeCalibration {
 public:
  // The bow projection is rejected when the chosen bow axis lies within this
  // angle of vertical at capture (its horizontal projection is then too short to
  // define forward). sin(8°) ≈ 0.139 — generous enough for any real mounting
  // slant, tight enough to catch a bow axis that actually points up or down.
  static constexpr double kMinBowHorizontal = 0.139;

  // Down (gravity) unit vector in the chip body frame, derived from the DMP
  // quaternion (w, x, y, z). Identity quaternion → (0, 0, -1).
  //
  // Frame: the DMP game rotation vector is 6-axis (accelerometer + gyroscope, no
  // magnetometer), so this and the bow axis are in the ICM-20948's
  // accelerometer/gyro frame. The on-board AK09916 magnetometer uses a different
  // frame (its Y and Z are inverted relative to the accel/gyro frame). That sign
  // difference is irrelevant here because the magnetometer is never read; it must
  // be handled if magnetic heading is added later (the DMP's 9-axis output applies
  // the alignment internally; raw magnetometer reads would need Y and Z negated).
  static Vec3 gravity_from_quat(double w, double x, double y, double z) {
    return Vec3{2.0 * (w * y - x * z), -2.0 * (w * x + y * z),
                2.0 * (x * x + y * y) - 1.0};
  }

  // Select which chip axis points toward the bow. Selecting a bow marks the device
  // calibrated; until then compute() is a passthrough that reproduces the
  // uncalibrated chip-frame angles (R6).
  void set_bow(Axis bow) {
    bow_ = axis_vec(bow);
    calibrated_ = true;
    revision_++;
  }

  bool calibrated() const { return calibrated_; }
  void mark_uncalibrated() {
    calibrated_ = false;
    revision_++;
  }

  // True once a level reference has actually been captured (or restored from
  // persistence). compute() stays in passthrough until both a bow is selected and
  // a level reference exists — selecting a bow alone must not emit angles against
  // an assumed-flat mount.
  bool leveled() const { return leveled_; }
  void set_leveled(bool v) { leveled_ = v; }

  // Derive the boat-frame axes (chip frame) from the bow axis and the level
  // reference. Returns false when the bow axis is within kMinBowHorizontal of
  // vertical, so its horizontal projection can't define forward.
  bool boat_axes(Vec3& fwd, Vec3& stbd, Vec3& down) const {
    down = normalized(g_chip0_);
    if (!forward_from_bow(down, fwd)) return false;
    stbd = cross(down, fwd);
    return true;
  }

  // Capture the current pose as level: store the chip-frame gravity vector as the
  // reference "down". Returns false (and stores nothing) if the selected bow axis
  // is vertical at this pose — the caller should tell the user to pick the axis
  // that faces the front of the boat.
  bool capture_level(double w, double x, double y, double z) {
    Vec3 down = normalized(gravity_from_quat(w, x, y, z));
    Vec3 fwd;
    if (!forward_from_bow(down, fwd)) return false;
    g_chip0_ = down;
    leveled_ = true;
    revision_++;
    return true;
  }

  // Clear to uncalibrated defaults.
  void reset() {
    bow_ = {1, 0, 0};
    g_chip0_ = {0, 0, -1};
    calibrated_ = false;
    leveled_ = false;
    revision_++;
  }

  // Compute boat-frame roll (heel, +starboard) and pitch (trim, +bow-up), radians.
  // Falls back to the uncalibrated passthrough until a bow is selected, a level
  // reference has been captured, and that reference yields a usable frame.
  void compute(double w, double x, double y, double z, double& roll,
               double& pitch) const {
    Vec3 fwd, stbd, down;
    if (!calibrated_ || !leveled_ || !boat_axes(fwd, stbd, down)) {
      passthrough(w, x, y, z, roll, pitch);
      return;
    }
    Vec3 g = normalized(gravity_from_quat(w, x, y, z));
    Vec3 gb = Vec3{dot(g, fwd), dot(g, stbd), dot(g, down)};
    roll_pitch_boat(gb, roll, pitch);
  }

  // Persisted reference gravity vector, chip frame (3 floats).
  Vec3 reference() const { return g_chip0_; }
  void set_reference(const Vec3& g) {
    // A degenerate (near-zero) stored reference reverts to chip-level, rather
    // than collapsing the frame to a silent no-op.
    g_chip0_ = norm(g) > 1e-6 ? normalized(g) : Vec3{0, 0, -1};
    revision_++;
  }

  // Increments on every state mutation; consumers (e.g. the rate differentiator)
  // use it to detect a calibration change and avoid differentiating across the
  // resulting one-sample discontinuity.
  uint32_t revision() const { return revision_; }

  // Boat-frame roll/pitch from a gravity (down) vector, with the calibrated sign
  // convention: heel-to-starboard → +roll, bow-up → +pitch.
  static void roll_pitch_boat(const Vec3& g, double& roll, double& pitch) {
    roll = std::atan2(g.y, g.z);
    pitch = std::atan2(-g.x, std::sqrt(g.y * g.y + g.z * g.z));
  }

 private:
  Vec3 bow_ = {1, 0, 0};
  Vec3 g_chip0_ = {0, 0, -1};
  bool calibrated_ = false;
  bool leveled_ = false;
  uint32_t revision_ = 0;

  // Project the bow axis onto the plane perpendicular to `down` and normalize it
  // to the boat forward axis. Returns false when the projection is shorter than
  // kMinBowHorizontal (bow axis too close to vertical to define forward). Shared
  // by capture_level (capture-time gate) and boat_axes (runtime derivation) so the
  // two cannot disagree on what counts as a usable bow.
  bool forward_from_bow(const Vec3& down, Vec3& fwd) const {
    Vec3 proj = Vec3{bow_.x - dot(bow_, down) * down.x,
                     bow_.y - dot(bow_, down) * down.y,
                     bow_.z - dot(bow_, down) * down.z};
    if (norm(proj) < kMinBowHorizontal) return false;
    fwd = normalized(proj);
    return true;
  }

  // Uncalibrated passthrough: the chip-frame roll/pitch as the MVP extracted them.
  static void passthrough(double w, double x, double y, double z, double& roll,
                          double& pitch) {
    roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    double sinp = 2.0 * (w * y - z * x);
    pitch = std::fabs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp)
                                   : std::asin(sinp);
  }
};

}  // namespace attitude_sensor

#endif  // ATTITUDE_SENSOR_SRC_ATTITUDE_CALIBRATION_H_
