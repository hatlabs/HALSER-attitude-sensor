// Host-side unit tests for the attitude calibration math (pio test -e native).

#include <unity.h>

#include <cmath>

#include "../../src/attitude_calibration.h"
#include "../../src/rate_filter.h"

using namespace attitude_sensor;

namespace {

constexpr double kDeg = M_PI / 180.0;

struct Quat {
  double w, x, y, z;
};

Quat quat_axis_angle(Vec3 axis, double angle) {
  Vec3 a = normalized(axis);
  double s = std::sin(angle / 2.0);
  return Quat{std::cos(angle / 2.0), a.x * s, a.y * s, a.z * s};
}

Quat quat_mul(const Quat& a, const Quat& b) {
  return Quat{a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
              a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
              a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
              a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

void assert_deg(double expected_deg, double actual_rad, double tol_deg) {
  TEST_ASSERT_DOUBLE_WITHIN(tol_deg, expected_deg, actual_rad / kDeg);
}

void assert_orthonormal_rh(const Vec3& fwd, const Vec3& stbd, const Vec3& down) {
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, norm(fwd));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, norm(stbd));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, norm(down));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, dot(fwd, stbd));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, dot(fwd, down));
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, dot(stbd, down));
  // Right-handed boat frame (x=fwd, y=stbd, z=down): x·(y×z) = +1.
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, dot(fwd, cross(stbd, down)));
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_gravity_from_quat_identity() {
  Vec3 g = AttitudeCalibration::gravity_from_quat(1, 0, 0, 0);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, g.x);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, g.y);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, -1.0, g.z);
}

// R6: uncalibrated passthrough reproduces the chip-frame angles. A 30° rotation
// about chip X reads as roll 30°, pitch 0.
void test_passthrough_matches_chip_angles() {
  AttitudeCalibration cal;  // not calibrated
  Quat q = quat_axis_angle({1, 0, 0}, 30 * kDeg);
  double roll, pitch;
  cal.compute(q.w, q.x, q.y, q.z, roll, pitch);
  assert_deg(30.0, roll, 0.05);
  assert_deg(0.0, pitch, 0.05);
}

void test_roll_pitch_boat_level() {
  double roll, pitch;
  AttitudeCalibration::roll_pitch_boat({0, 0, 1}, roll, pitch);
  assert_deg(0.0, roll, 1e-6);
  assert_deg(0.0, pitch, 1e-6);
}

// Heel to starboard tilts gravity toward +y → positive roll.
void test_roll_pitch_boat_heel_starboard() {
  double roll, pitch;
  AttitudeCalibration::roll_pitch_boat({0, std::sin(20 * kDeg), std::cos(20 * kDeg)},
                                       roll, pitch);
  assert_deg(20.0, roll, 0.01);
  assert_deg(0.0, pitch, 0.01);
}

// Bow up tilts gravity toward -x → positive pitch.
void test_roll_pitch_boat_bow_up() {
  double roll, pitch;
  AttitudeCalibration::roll_pitch_boat(
      {-std::sin(10 * kDeg), 0, std::cos(10 * kDeg)}, roll, pitch);
  assert_deg(10.0, pitch, 0.01);
  assert_deg(0.0, roll, 0.01);
}

// The bow dropdown is the six signed chip axes; every label round-trips.
void test_bow_axes_catalog() {
  unsigned n;
  const Axis* axes = bow_axes(n);
  TEST_ASSERT_EQUAL_UINT(6, n);
  char buf[16];
  for (unsigned i = 0; i < n; i++) {
    bow_label(axes[i], buf, sizeof(buf));
    Axis parsed;
    TEST_ASSERT_TRUE(parse_bow(buf, parsed));
    TEST_ASSERT_TRUE(parsed == axes[i]);
  }
}

void test_parse_bow() {
  Axis bow;
  TEST_ASSERT_TRUE(parse_bow("Bow +Y", bow));
  TEST_ASSERT_TRUE(bow == Axis::kPlusY);
  TEST_ASSERT_TRUE(parse_bow("Bow -Z", bow));
  TEST_ASSERT_TRUE(bow == Axis::kMinusZ);
  TEST_ASSERT_FALSE(parse_bow("Uncalibrated", bow));
  TEST_ASSERT_FALSE(parse_bow("Bow +Q", bow));
}

// For a flat mount, each horizontal bow axis yields a right-handed boat frame
// whose forward axis is exactly the chosen chip axis.
void test_horizontal_bows_right_handed() {
  const Axis bows[] = {Axis::kPlusX, Axis::kMinusX, Axis::kPlusY, Axis::kMinusY};
  for (Axis b : bows) {
    AttitudeCalibration cal;
    cal.set_bow(b);
    TEST_ASSERT_TRUE(cal.capture_level(1, 0, 0, 0));  // flat: chip -Z is down
    Vec3 fwd, stbd, down;
    TEST_ASSERT_TRUE(cal.boat_axes(fwd, stbd, down));
    assert_orthonormal_rh(fwd, stbd, down);
    Vec3 a = axis_vec(b);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, dot(fwd, a));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -1.0, down.z);  // down is chip -Z
  }
}

// The new model's core: the bow axis only needs to be approximately right. With a
// slanted level pose, forward is the bow axis projected onto the horizontal plane,
// the frame stays orthonormal/right-handed, and the captured pose reads 0/0.
void test_bow_projection_offaxis_frame() {
  AttitudeCalibration cal;
  cal.set_bow(Axis::kPlusX);
  // Board mounted slanted: 12° about X and 9° about Y at level.
  Quat level = quat_mul(quat_axis_angle({1, 0, 0}, 12 * kDeg),
                        quat_axis_angle({0, 1, 0}, 9 * kDeg));
  TEST_ASSERT_TRUE(cal.capture_level(level.w, level.x, level.y, level.z));

  Vec3 fwd, stbd, down;
  TEST_ASSERT_TRUE(cal.boat_axes(fwd, stbd, down));
  assert_orthonormal_rh(fwd, stbd, down);
  // Forward still points predominantly along the chosen +X bow axis.
  TEST_ASSERT_TRUE(dot(fwd, Vec3{1, 0, 0}) > 0.9);

  double roll, pitch;
  cal.compute(level.w, level.x, level.y, level.z, roll, pitch);
  assert_deg(0.0, roll, 0.02);
  assert_deg(0.0, pitch, 0.02);
}

// A bulkhead mount where the bow axis is the chip's Z works, but only once a level
// pose that puts Z horizontal is captured. Capturing while Z is vertical (bow
// pointing up/down) is degenerate and must be refused.
void test_vertical_bow_capture_rejected() {
  AttitudeCalibration cal;
  cal.set_bow(Axis::kPlusZ);
  // Flat on the bench: chip +Z points straight up, so it cannot be the bow.
  TEST_ASSERT_FALSE(cal.capture_level(1, 0, 0, 0));

  // Mounted on a bulkhead: a 90° roll about X puts chip +Z horizontal (gravity
  // along chip -Y). Now +Z is a valid bow.
  Quat bulkhead = quat_axis_angle({1, 0, 0}, 90 * kDeg);
  TEST_ASSERT_TRUE(
      cal.capture_level(bulkhead.w, bulkhead.x, bulkhead.y, bulkhead.z));
  Vec3 fwd, stbd, down;
  TEST_ASSERT_TRUE(cal.boat_axes(fwd, stbd, down));
  assert_orthonormal_rh(fwd, stbd, down);
  double roll, pitch;
  cal.compute(bulkhead.w, bulkhead.x, bulkhead.y, bulkhead.z, roll, pitch);
  assert_deg(0.0, roll, 0.02);
  assert_deg(0.0, pitch, 0.02);
}

// After capturing a slanted level pose, that exact pose reads 0/0.
void test_capture_level_zeroes_slanted_mount() {
  AttitudeCalibration cal;
  cal.set_bow(Axis::kPlusX);
  // Board sits slanted: 15° about X and 8° about Y at "level".
  Quat slant = quat_mul(quat_axis_angle({1, 0, 0}, 15 * kDeg),
                        quat_axis_angle({0, 1, 0}, 8 * kDeg));
  TEST_ASSERT_TRUE(cal.capture_level(slant.w, slant.x, slant.y, slant.z));
  double roll, pitch;
  cal.compute(slant.w, slant.x, slant.y, slant.z, roll, pitch);
  assert_deg(0.0, roll, 0.02);
  assert_deg(0.0, pitch, 0.02);
}

// P0 regression: a captured reference must survive a reboot (arbitrary new yaw
// origin). A world-vertical yaw premultiply leaves boat-frame roll/pitch ~0/0,
// because only the (yaw-independent) gravity direction feeds roll/pitch.
void test_reboot_stability_under_yaw_shift() {
  AttitudeCalibration cal;
  cal.set_bow(Axis::kPlusX);
  Quat q0 = quat_mul(quat_axis_angle({1, 0, 0}, 25 * kDeg),
                     quat_axis_angle({0, 1, 0}, 12 * kDeg));
  TEST_ASSERT_TRUE(cal.capture_level(q0.w, q0.x, q0.y, q0.z));

  // Simulate persistence + reboot: serialize the reference, rebuild, reload.
  Vec3 saved = cal.reference();
  AttitudeCalibration rebooted;
  rebooted.set_bow(Axis::kPlusX);
  rebooted.set_reference(saved);
  rebooted.set_leveled(true);  // persisted level flag restored on boot

  // New boot: same physical pose, arbitrary new yaw origin (world-z premultiply).
  Quat yaw = quat_axis_angle({0, 0, 1}, 133 * kDeg);
  Quat q_reboot = quat_mul(yaw, q0);
  double roll, pitch;
  rebooted.compute(q_reboot.w, q_reboot.x, q_reboot.y, q_reboot.z, roll, pitch);
  assert_deg(0.0, roll, 0.01);
  assert_deg(0.0, pitch, 0.01);
}

// Negative control: a full-quaternion reference (q_ref⁻¹ ⊗ q) drifts under the
// same yaw shift — proving the tilt-only storage choice is what confers stability.
void test_negative_control_full_quat_reference_drifts() {
  Quat q0 = quat_mul(quat_axis_angle({1, 0, 0}, 25 * kDeg),
                     quat_axis_angle({0, 1, 0}, 12 * kDeg));
  Quat q0_inv = Quat{q0.w, -q0.x, -q0.y, -q0.z};  // unit quaternion conjugate

  Quat yaw = quat_axis_angle({0, 0, 1}, 133 * kDeg);
  Quat q_reboot = quat_mul(yaw, q0);

  Quat corrected = quat_mul(q0_inv, q_reboot);
  // Extract roll/pitch from the full-quaternion-corrected result.
  double roll, pitch;
  AttitudeCalibration::roll_pitch_boat(
      AttitudeCalibration::gravity_from_quat(corrected.w, corrected.x,
                                             corrected.y, corrected.z),
      roll, pitch);
  // It should be far from level — the yaw leaked into tilt.
  double err_deg = std::sqrt(roll * roll + pitch * pitch) / kDeg;
  TEST_ASSERT_TRUE(err_deg > 20.0);
}

// The level reference is stored in the chip frame, so changing the bow selection
// keeps the same physical pose reading ~0/0 (no re-capture needed): at the
// reference pose gravity equals "down", which projects to (0,0,1) for any bow.
void test_reference_survives_bow_change() {
  AttitudeCalibration cal;
  cal.set_bow(Axis::kPlusX);
  Quat pose = quat_mul(quat_axis_angle({1, 0, 0}, 7 * kDeg),
                       quat_axis_angle({0, 1, 0}, 4 * kDeg));
  TEST_ASSERT_TRUE(cal.capture_level(pose.w, pose.x, pose.y, pose.z));
  cal.set_bow(Axis::kPlusY);  // user changes the dropdown
  double roll, pitch;
  cal.compute(pose.w, pose.x, pose.y, pose.z, roll, pitch);
  assert_deg(0.0, roll, 0.02);
  assert_deg(0.0, pitch, 0.02);
}

// The sign convention must survive the full compute() pipeline (gravity → boat
// projection → roll_pitch_boat) for bow axes other than +X — the host suite
// otherwise only pins signs at the 0/0 reference pose. Flat mount: a positive
// rotation about the chip axis that becomes "forward" is a heel to starboard and
// must read +roll; the orthogonal tilt must read +pitch, with the other near 0.
void test_signed_heel_trim_per_bow() {
  double roll, pitch;
  {  // bow = +X: forward = chip +X
    AttitudeCalibration cal;
    cal.set_bow(Axis::kPlusX);
    TEST_ASSERT_TRUE(cal.capture_level(1, 0, 0, 0));
    Quat heel = quat_axis_angle({1, 0, 0}, 15 * kDeg);
    cal.compute(heel.w, heel.x, heel.y, heel.z, roll, pitch);
    assert_deg(15.0, roll, 0.05);
    assert_deg(0.0, pitch, 0.05);
    Quat bow_up = quat_axis_angle({0, 1, 0}, -15 * kDeg);
    cal.compute(bow_up.w, bow_up.x, bow_up.y, bow_up.z, roll, pitch);
    assert_deg(15.0, pitch, 0.05);
    assert_deg(0.0, roll, 0.05);
  }
  {  // bow = +Y: forward = chip +Y, starboard = chip +X
    AttitudeCalibration cal;
    cal.set_bow(Axis::kPlusY);
    TEST_ASSERT_TRUE(cal.capture_level(1, 0, 0, 0));
    Quat heel = quat_axis_angle({0, 1, 0}, 15 * kDeg);
    cal.compute(heel.w, heel.x, heel.y, heel.z, roll, pitch);
    assert_deg(15.0, roll, 0.05);
    assert_deg(0.0, pitch, 0.05);
  }
}

// Selecting a bow without ever capturing Set Level must NOT emit calibrated
// angles against an assumed-flat mount — it stays a passthrough, identical to a
// fully-uncalibrated device, until a level reference is captured.
void test_bow_without_level_is_passthrough() {
  AttitudeCalibration gated;
  gated.set_bow(Axis::kPlusX);  // bow selected, never leveled
  TEST_ASSERT_TRUE(gated.calibrated());
  TEST_ASSERT_FALSE(gated.leveled());

  AttitudeCalibration raw;  // fully uncalibrated
  Quat q = quat_mul(quat_axis_angle({1, 0, 0}, 20 * kDeg),
                    quat_axis_angle({0, 1, 0}, 15 * kDeg));
  double rg, pg, rr, pr;
  gated.compute(q.w, q.x, q.y, q.z, rg, pg);
  raw.compute(q.w, q.x, q.y, q.z, rr, pr);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, rr, rg);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, pr, pg);
}

// -Z bulkhead mount: the chip's -Z faces the bow. A -90° roll about X puts -Z
// horizontal; the frame must be right-handed with forward along -Z.
void test_minus_z_bulkhead_right_handed() {
  AttitudeCalibration cal;
  cal.set_bow(Axis::kMinusZ);
  Quat bulkhead = quat_axis_angle({1, 0, 0}, -90 * kDeg);
  TEST_ASSERT_TRUE(
      cal.capture_level(bulkhead.w, bulkhead.x, bulkhead.y, bulkhead.z));
  Vec3 fwd, stbd, down;
  TEST_ASSERT_TRUE(cal.boat_axes(fwd, stbd, down));
  assert_orthonormal_rh(fwd, stbd, down);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, dot(fwd, axis_vec(Axis::kMinusZ)));
  double roll, pitch;
  cal.compute(bulkhead.w, bulkhead.x, bulkhead.y, bulkhead.z, roll, pitch);
  assert_deg(0.0, roll, 0.02);
  assert_deg(0.0, pitch, 0.02);
}

// The kMinBowHorizontal gate (~sin 8°): a bow axis tilted just under 8° from
// vertical is rejected; just over is accepted. Tilting a +Z bow α° about X makes
// its horizontal projection sin(α), so α brackets the threshold directly.
void test_bow_near_vertical_boundary() {
  AttitudeCalibration cal;
  cal.set_bow(Axis::kPlusZ);
  Quat under = quat_axis_angle({1, 0, 0}, 7 * kDeg);  // sin 7° ≈ 0.122 < 0.139
  TEST_ASSERT_FALSE(cal.capture_level(under.w, under.x, under.y, under.z));
  Quat over = quat_axis_angle({1, 0, 0}, 9 * kDeg);  // sin 9° ≈ 0.156 > 0.139
  TEST_ASSERT_TRUE(cal.capture_level(over.w, over.x, over.y, over.z));
}

// A calibrated device whose stored reference makes the bow axis vertical (down ∥
// bow) yields no usable frame; compute() must fall back to the finite passthrough,
// not emit NaN. Here down = +X is parallel to the +X bow, so boat_axes() fails and
// the result is the chip-frame angle (30° about X reads as 30° passthrough roll).
void test_degenerate_frame_falls_back_to_passthrough() {
  AttitudeCalibration cal;
  cal.set_bow(Axis::kPlusX);
  cal.set_reference({1, 0, 0});
  cal.set_leveled(true);  // leveled, but the reference makes the frame degenerate
  Quat q = quat_axis_angle({1, 0, 0}, 30 * kDeg);
  double roll, pitch;
  cal.compute(q.w, q.x, q.y, q.z, roll, pitch);
  TEST_ASSERT_FALSE(std::isnan(roll));
  TEST_ASSERT_FALSE(std::isnan(pitch));
  assert_deg(30.0, roll, 0.05);  // passthrough chip angle, not a calibrated value
  assert_deg(0.0, pitch, 0.05);
}

// A near-zero stored reference reverts to chip-level (0,0,-1) rather than
// collapsing the frame to a silent no-op.
void test_set_reference_degenerate_reverts() {
  AttitudeCalibration cal;
  cal.set_reference({0, 0, 0});
  Vec3 r = cal.reference();
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, r.x);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, r.y);
  TEST_ASSERT_DOUBLE_WITHIN(1e-9, -1.0, r.z);
}

// revision() bumps on every real state change but NOT on a rejected capture — the
// imu.h rate differentiator re-seeds on revision change, so a rejected Set Level
// must not look like a calibration change (which would inject a spurious rate).
void test_revision_contract() {
  AttitudeCalibration cal;
  uint32_t r0 = cal.revision();
  cal.set_bow(Axis::kPlusX);
  TEST_ASSERT_TRUE(cal.revision() > r0);
  uint32_t r1 = cal.revision();
  TEST_ASSERT_TRUE(cal.capture_level(1, 0, 0, 0));  // +X flat: accepted
  TEST_ASSERT_TRUE(cal.revision() > r1);

  cal.set_bow(Axis::kPlusZ);
  uint32_t r2 = cal.revision();
  TEST_ASSERT_FALSE(cal.capture_level(1, 0, 0, 0));  // +Z flat: rejected
  TEST_ASSERT_EQUAL_UINT(r2, cal.revision());        // no bump on rejection
}

// Near the pitch singularity the result stays finite (no NaN).
void test_near_singular_pitch_finite() {
  double roll, pitch;
  AttitudeCalibration::roll_pitch_boat(
      {-std::sin(88 * kDeg), 0, std::cos(88 * kDeg)}, roll, pitch);
  TEST_ASSERT_FALSE(std::isnan(roll));
  TEST_ASSERT_FALSE(std::isnan(pitch));
  assert_deg(88.0, pitch, 0.1);
}

// The rate smoother reseeds from the fresh sample when the previous value is
// non-finite, so a NaN emitted during an uncalibrated/settling gap does not
// poison the exponential filter once output resumes.
void test_seed_safe_smooth() {
  const float a = 0.3f;
  // Finite previous: standard exponential blend.
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.3f * 5.0f + 0.7f * 1.0f,
                           seed_safe_smooth(1.0f, 5.0f, a));
  // Non-finite previous (NaN or Inf): reseed from the fresh sample.
  TEST_ASSERT_EQUAL_FLOAT(5.0f, seed_safe_smooth(NAN, 5.0f, a));
  TEST_ASSERT_EQUAL_FLOAT(5.0f, seed_safe_smooth(INFINITY, 5.0f, a));
  // valid -> NaN gap -> valid: the first post-gap output is the fresh sample,
  // not a NaN-poisoned blend, and subsequent samples blend normally.
  float s = seed_safe_smooth(2.0f, 4.0f, a);      // finite blend
  s = NAN;                                         // gap emits NaN
  s = seed_safe_smooth(s, 6.0f, a);                // resume
  TEST_ASSERT_EQUAL_FLOAT(6.0f, s);
  s = seed_safe_smooth(s, 6.0f, a);                // now blends, stays finite
  TEST_ASSERT_TRUE(std::isfinite(s));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_seed_safe_smooth);
  RUN_TEST(test_gravity_from_quat_identity);
  RUN_TEST(test_passthrough_matches_chip_angles);
  RUN_TEST(test_roll_pitch_boat_level);
  RUN_TEST(test_roll_pitch_boat_heel_starboard);
  RUN_TEST(test_roll_pitch_boat_bow_up);
  RUN_TEST(test_bow_axes_catalog);
  RUN_TEST(test_parse_bow);
  RUN_TEST(test_horizontal_bows_right_handed);
  RUN_TEST(test_bow_projection_offaxis_frame);
  RUN_TEST(test_vertical_bow_capture_rejected);
  RUN_TEST(test_capture_level_zeroes_slanted_mount);
  RUN_TEST(test_reboot_stability_under_yaw_shift);
  RUN_TEST(test_negative_control_full_quat_reference_drifts);
  RUN_TEST(test_reference_survives_bow_change);
  RUN_TEST(test_signed_heel_trim_per_bow);
  RUN_TEST(test_bow_without_level_is_passthrough);
  RUN_TEST(test_minus_z_bulkhead_right_handed);
  RUN_TEST(test_bow_near_vertical_boundary);
  RUN_TEST(test_degenerate_frame_falls_back_to_passthrough);
  RUN_TEST(test_set_reference_degenerate_reverts);
  RUN_TEST(test_revision_contract);
  RUN_TEST(test_near_singular_pitch_finite);
  return UNITY_END();
}
