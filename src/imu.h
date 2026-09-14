#ifndef ATTITUDE_SENSOR_SRC_IMU_H_
#define ATTITUDE_SENSOR_SRC_IMU_H_

#include <ICM_20948.h>
#include <Wire.h>

#include <cmath>

#include "attitude_calibration.h"
#include "rate_filter.h"
#include "sensesp.h"
#include "sensesp/system/observablevalue.h"

namespace attitude_sensor {

// ICM-20948 attitude sensor.
//
// Uses the chip's on-board DMP in 6-axis "game rotation vector" mode, which
// fuses accelerometer and gyroscope into a drift-stable orientation without
// the magnetometer.
//
// Roll and pitch are produced by the shared AttitudeCalibration (which derives
// them from the gravity vector, so they are reboot-stable and, once calibrated,
// expressed as boat heel/trim). Roll/pitch rates are the numerical derivatives
// of those calibrated angles. Yaw is relative (no magnetic reference); only its
// derivative (rate of turn) is exposed, computed as the angular velocity about
// the gravity direction and so unaffected by calibration and by mounting.
class AttitudeSensor {
 public:
  AttitudeSensor(TwoWire* i2c, AttitudeCalibration* calibration,
                 uint8_t ad0_val = 1, unsigned int interval_ms = 100)
      : i2c_{i2c}, calibration_{calibration}, interval_ms_{interval_ms} {
    // Bring up the IMU and DMP, retrying a few times: a transient I2C glitch
    // during boot can otherwise leave the sensor dead for the whole session.
    // begin() answers at 0x68 or 0x69 depending on the AD0 pin level (breakouts
    // differ), so try the requested address first, then the other.
    bool ok = false;
    for (int attempt = 1; attempt <= kInitMaxAttempts; ++attempt) {
      if ((imu_.begin(*i2c_, ad0_val) == ICM_20948_Stat_Ok ||
           imu_.begin(*i2c_, ad0_val ? 0 : 1) == ICM_20948_Stat_Ok) &&
          init_dmp()) {
        ok = true;
        break;
      }
      ESP_LOGW("AttitudeSensor", "ICM-20948 init attempt %d/%d failed: %s",
               attempt, kInitMaxAttempts, imu_.statusString());
      delay(kInitRetryDelayMs);
    }
    if (!ok) {
      ESP_LOGE("AttitudeSensor", "ICM-20948 init failed after %d attempts",
               kInitMaxAttempts);
      return;
    }
    ESP_LOGI("AttitudeSensor", "ICM-20948 DMP initialized");
    sensesp::event_loop()->onRepeat(interval_ms_, [this]() { read(); });
  }

  // Attitude, radians. NaN until a calibrated, settled reading exists, so
  // consumers see "no data" (Signal K null / N2K not-available) rather than a
  // fabricated flat 0/0 at boot or if the sensor never initializes.
  sensesp::ObservableValue<float> roll{NAN};
  sensesp::ObservableValue<float> pitch{NAN};

  // Attitude rates, radians/second. NaN until the first valid sample (same
  // reasoning); the rate smoother reseeds from the first finite value.
  sensesp::ObservableValue<float> roll_rate{NAN};
  sensesp::ObservableValue<float> pitch_rate{NAN};
  sensesp::ObservableValue<float> yaw_rate{NAN};

  // True once a valid quaternion has been seen and the DMP has had time to
  // converge (~40 s). Set Level capture should be gated on this.
  bool dmp_ready() const {
    return have_quat_ && (millis() - dmp_start_ms_) >= kDmpSettleMs;
  }

  // Latest raw chip quaternion (w, x, y, z). Returns false if none yet.
  bool latest_quat(double& w, double& x, double& y, double& z) const {
    if (!have_quat_) return false;
    w = q0_;
    x = q1_;
    y = q2_;
    z = q3_;
    return true;
  }

 private:
  ICM_20948_I2C imu_;
  TwoWire* i2c_;
  AttitudeCalibration* calibration_;
  unsigned int interval_ms_;

  // Latest raw chip quaternion.
  double q0_ = 1, q1_ = 0, q2_ = 0, q3_ = 0;
  bool have_quat_ = false;
  uint32_t dmp_start_ms_ = 0;

  double prev_roll_ = 0;
  double prev_pitch_ = 0;
  double pq0_ = 1, pq1_ = 0, pq2_ = 0, pq3_ = 0;
  uint32_t prev_us_ = 0;
  // The rate of turn comes from the raw quaternions, so a calibration change
  // does not disturb it; the roll/pitch rates differentiate calibrated angles
  // and do have to be reseeded across one. Hence two separate seed flags.
  bool have_prev_quat_ = false;
  bool have_prev_angles_ = false;
  uint32_t last_cal_revision_ = 0;
  uint32_t last_log_ms_ = 0;

  // Retry IMU/DMP bring-up so a transient boot-time I2C glitch doesn't leave the
  // sensor dead for the whole session.
  static constexpr int kInitMaxAttempts = 5;
  static constexpr uint32_t kInitRetryDelayMs = 100;
  // DMP game-rotation-vector convergence time after boot (observed on device).
  static constexpr uint32_t kDmpSettleMs = 40000;
  // DMP Quat6 ODR divider. Keep at 0. A non-zero divider does not reduce FIFO
  // frame production -- it only strips the Quat6 payload from most frames, so
  // the drain loop wastes ~10x the I2C reads to reach each quaternion.
  static constexpr int kDmpOdrDivider = 0;
  // Light low-pass on the differentiated rates to tame quantization noise.
  static constexpr float kRateSmoothing = 0.3f;

  bool init_dmp() {
    bool ok = true;
    ok &= (imu_.initializeDMP() == ICM_20948_Stat_Ok);
    ok &= (imu_.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) ==
           ICM_20948_Stat_Ok);
    ok &= (imu_.setDMPODRrate(DMP_ODR_Reg_Quat6, kDmpOdrDivider) ==
           ICM_20948_Stat_Ok);
    ok &= (imu_.enableFIFO() == ICM_20948_Stat_Ok);
    ok &= (imu_.enableDMP() == ICM_20948_Stat_Ok);
    ok &= (imu_.resetDMP() == ICM_20948_Stat_Ok);
    ok &= (imu_.resetFIFO() == ICM_20948_Stat_Ok);
    return ok;
  }

  static double wrap_pi(double a) {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
  }

  void read() {
    icm_20948_DMP_data_t data;
    bool got_quat = false;
    double q1 = 0, q2 = 0, q3 = 0;

    // Drain the FIFO, keeping the most recent quaternion.
    do {
      imu_.readDMPdataFromFIFO(&data);
      if ((imu_.status == ICM_20948_Stat_Ok ||
           imu_.status == ICM_20948_Stat_FIFOMoreDataAvail) &&
          (data.header & DMP_header_bitmap_Quat6) > 0) {
        // Q1..Q3 are signed and scaled by 2^30.
        q1 = static_cast<double>(data.Quat6.Data.Q1) / 1073741824.0;
        q2 = static_cast<double>(data.Quat6.Data.Q2) / 1073741824.0;
        q3 = static_cast<double>(data.Quat6.Data.Q3) / 1073741824.0;
        got_quat = true;
      }
    } while (imu_.status == ICM_20948_Stat_FIFOMoreDataAvail);

    if (!got_quat) {
      return;
    }

    double q0sq = 1.0 - (q1 * q1 + q2 * q2 + q3 * q3);
    double q0 = q0sq > 0.0 ? std::sqrt(q0sq) : 0.0;

    q0_ = q0;
    q1_ = q1;
    q2_ = q2;
    q3_ = q3;
    if (!have_quat_) {
      have_quat_ = true;
      dmp_start_ms_ = millis();
    }

    // Calibrated boat-frame roll/pitch (passthrough until calibrated).
    double r, p;
    calibration_->compute(q0, q1, q2, q3, r, p);

    // Publish boat attitude only once calibrated, leveled, and the DMP has
    // settled; until then emit NaN (Signal K null / N2K "not available") so the
    // bus never carries a heel/trim derived from the raw, unmapped chip frame or
    // from an unconverged DMP (the ~40 s window after every power-up).
    bool attitude_valid = calibration_->calibrated() &&
                          calibration_->leveled() && dmp_ready();
    roll.set(attitude_valid ? static_cast<float>(r) : NAN);
    pitch.set(attitude_valid ? static_cast<float>(p) : NAN);

    // Re-seed the roll/pitch differentiator across a calibration change so the
    // one-sample angle discontinuity doesn't emit a spurious rate spike.
    uint32_t cal_rev = calibration_->revision();
    if (cal_rev != last_cal_revision_) {
      last_cal_revision_ = cal_rev;
      have_prev_angles_ = false;
    }

    uint32_t now_us = micros();
    double dt = have_prev_quat_ ? (now_us - prev_us_) / 1e6 : 0.0;
    if (dt > 0) {
      // Rate of turn is the angular velocity about the vessel vertical, taken
      // from the two quaternions and the gravity direction they carry. That
      // makes it calibration-independent and correct on every mounting; see
      // AttitudeCalibration::rate_of_turn.
      float yr = static_cast<float>(AttitudeCalibration::rate_of_turn(
          pq0_, pq1_, pq2_, pq3_, q0, q1, q2, q3, dt));
      yaw_rate.set(seed_safe_smooth(yaw_rate.get(), yr, kRateSmoothing));
      // Roll/pitch rates are only meaningful once the boat frame is known;
      // emit NaN otherwise (same rule as the attitude above).
      if (attitude_valid && have_prev_angles_) {
        float rr = static_cast<float>(wrap_pi(r - prev_roll_) / dt);
        float pr = static_cast<float>(wrap_pi(p - prev_pitch_) / dt);
        roll_rate.set(seed_safe_smooth(roll_rate.get(), rr, kRateSmoothing));
        pitch_rate.set(seed_safe_smooth(pitch_rate.get(), pr, kRateSmoothing));
      } else if (!attitude_valid) {
        roll_rate.set(NAN);
        pitch_rate.set(NAN);
      }
    }
    prev_roll_ = r;
    prev_pitch_ = p;
    pq0_ = q0;
    pq1_ = q1;
    pq2_ = q2;
    pq3_ = q3;
    prev_us_ = now_us;
    have_prev_quat_ = true;
    have_prev_angles_ = true;

    // Throttled debug view of the calibrated attitude (the 10 Hz read rate would
    // otherwise flood the log).
    uint32_t now_ms = millis();
    if (now_ms - last_log_ms_ >= 1000) {
      last_log_ms_ = now_ms;
      ESP_LOGD("AttitudeSensor",
               "roll=%.1f pitch=%.1f deg | rate r=%.2f p=%.2f y=%.2f rad/s | %s",
               r * 180.0 / M_PI, p * 180.0 / M_PI, roll_rate.get(),
               pitch_rate.get(), yaw_rate.get(),
               calibration_->calibrated() ? "calibrated" : "uncalibrated");
    }
  }
};

}  // namespace attitude_sensor

#endif  // ATTITUDE_SENSOR_SRC_IMU_H_
