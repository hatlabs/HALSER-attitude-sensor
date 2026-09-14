// Host-side unit tests for the rate filter (pio test -e native).

#include <unity.h>

#include <cmath>

#include "../../src/rate_filter.h"

using namespace attitude_sensor;

void setUp() {}
void tearDown() {}

// Standard exponential blend with finite previous value.
void test_finite_blend() {
  float result = seed_safe_smooth(1.0f, 5.0f, 0.3f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.3f * 5.0f + 0.7f * 1.0f, result);
}

// NaN previous reseeds from the fresh sample (no NaN poisoning).
void test_reseed_on_nan() {
  TEST_ASSERT_EQUAL_FLOAT(5.0f, seed_safe_smooth(NAN, 5.0f, 0.3f));
}

// Infinity previous also reseeds.
void test_reseed_on_inf() {
  TEST_ASSERT_EQUAL_FLOAT(5.0f, seed_safe_smooth(INFINITY, 5.0f, 0.3f));
}

// Passthrough for tau = 0: alpha=0 means output = prev, so fresh is ignored.
// But with alpha=1 the filter passes fresh through entirely.
void test_alpha_one_passthrough() {
  TEST_ASSERT_EQUAL_FLOAT(7.0f, seed_safe_smooth(3.0f, 7.0f, 1.0f));
}

// Alpha=0: output is entirely the previous value.
void test_alpha_zero_hold() {
  TEST_ASSERT_EQUAL_FLOAT(3.0f, seed_safe_smooth(3.0f, 7.0f, 0.0f));
}

// NaN gap followed by resume: first post-gap sample is the fresh value,
// subsequent samples blend normally.
void test_gap_resume_sequence() {
  float s = seed_safe_smooth(2.0f, 4.0f, 0.3f);  // finite blend
  s = NAN;                                         // gap
  s = seed_safe_smooth(s, 6.0f, 0.3f);             // resume: reseeds
  TEST_ASSERT_EQUAL_FLOAT(6.0f, s);
  s = seed_safe_smooth(s, 6.0f, 0.3f);             // now blends
  TEST_ASSERT_TRUE(std::isfinite(s));
  TEST_ASSERT_EQUAL_FLOAT(6.0f, s);                // same input → converges
}

// Sign convention: positive rate of turn is starboard (clockwise from above).
// This is a documentation test — the rate_filter itself is sign-transparent,
// but pinning the expectation here ensures nothing in the pipeline flips it.
void test_sign_convention_passthrough() {
  // A positive yaw rate (starboard) passes through unchanged.
  float positive_rot = seed_safe_smooth(NAN, 0.1f, 0.3f);
  TEST_ASSERT_TRUE(positive_rot > 0.0f);
  // A negative yaw rate (port) passes through unchanged.
  float negative_rot = seed_safe_smooth(NAN, -0.1f, 0.3f);
  TEST_ASSERT_TRUE(negative_rot < 0.0f);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_finite_blend);
  RUN_TEST(test_reseed_on_nan);
  RUN_TEST(test_reseed_on_inf);
  RUN_TEST(test_alpha_one_passthrough);
  RUN_TEST(test_alpha_zero_hold);
  RUN_TEST(test_gap_resume_sequence);
  RUN_TEST(test_sign_convention_passthrough);
  return UNITY_END();
}
