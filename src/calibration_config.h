#ifndef ATTITUDE_SENSOR_SRC_CALIBRATION_CONFIG_H_
#define ATTITUDE_SENSOR_SRC_CALIBRATION_CONFIG_H_

#include <vector>

#include "attitude_calibration.h"
#include "imu.h"
#include "sensesp.h"
#include "sensesp/system/observablevalue.h"
#include "sensesp/system/saveable.h"
#include "sensesp/system/serializable.h"
#include "sensesp/ui/ui_controls.h"

// Web-UI calibration control for the ICM-20948. A single card holds the bow
// direction and the level reference: the FileSystemSaveable + Serializable +
// ConfigSchema pattern persists to flash and applies its
// effect to the shared AttitudeCalibration on save/load. The bow-axis catalog
// lives in attitude_calibration.h (single source of truth for both the parser and
// the dropdown enum below).

namespace attitude_sensor {

// Drive the status surface from the actual calibration state, so it stays correct
// regardless of when it was written (e.g. after a reboot, once the level reference
// has loaded).
inline void update_cal_status(const AttitudeCalibration* cal,
                              sensesp::ObservableValue<String>* status) {
  if (!status) return;
  if (!cal->calibrated()) {
    status->set("Uncalibrated");
  } else if (!cal->leveled()) {
    status->set("Bow set — Save with the boat level to finish");
  } else {
    status->set("Calibrated");
  }
}

// Dropdown labels: "Uncalibrated" plus the six bow axes, with labels generated
// from the shared axis table.
inline std::vector<String> BowLabels() {
  std::vector<String> labels;
  labels.push_back(kBowUncalibrated);
  unsigned n;
  const Axis* axes = bow_axes(n);
  char buf[16];
  for (unsigned i = 0; i < n; i++) {
    bow_label(axes[i], buf, sizeof(buf));
    labels.push_back(buf);
  }
  return labels;
}

// Single calibration card: bow direction plus the level reference. Selecting a
// bow axis marks the device calibrated; "Uncalibrated" reverts to passthrough.
// Saving the card -- with a bow selected and the IMU settled, the boat at rest on
// an even keel -- captures the current pose as the level reference (there is no
// separate capture control; Save is the trigger). The reference vector is shown
// read-only. Persists the bow, the reference vector, and the leveled flag.
class CalibrationConfig : public sensesp::FileSystemSaveable,
                          public sensesp::Serializable {
 public:
  CalibrationConfig(AttitudeCalibration* cal, AttitudeSensor* imu,
                    sensesp::ObservableValue<String>* status, String config_path)
      : sensesp::FileSystemSaveable(config_path),
        calibration_{cal},
        imu_{imu},
        status_{status} {
    load();
    apply();
  }

  bool to_json(JsonObject& doc) override {
    doc["bow"] = bow_;
    doc["gx"] = gx_;
    doc["gy"] = gy_;
    doc["gz"] = gz_;
    doc["leveled"] = leveled_;
    return true;
  }

  bool from_json(const JsonObject& config) override {
    if (config["bow"].is<String>()) bow_ = config["bow"].as<String>();
    if (config["gx"].is<float>()) gx_ = config["gx"];
    if (config["gy"].is<float>()) gy_ = config["gy"];
    if (config["gz"].is<float>()) gz_ = config["gz"];
    leveled_ = config["leveled"].is<bool>() && config["leveled"].as<bool>();
    return true;
  }

  // Apply the bow selection and, when a bow is set, capture the current pose as
  // level. A real bow with the IMU settled and the bow axis horizontal succeeds;
  // otherwise a guiding status is set and the prior reference is kept.
  bool save() override {
    apply_bow();
    bool captured = false;
    if (!calibration_->calibrated()) {
      leveled_ = false;
      calibration_->set_leveled(false);
    } else {
      captured = capture_level();
    }
    reconcile_leveled();
    // capture_level() has already put guidance on the status surface when it
    // failed; don't overwrite that with the generic state text.
    if (captured || !calibration_->calibrated()) {
      update_cal_status(calibration_, status_);
    }
    return sensesp::FileSystemSaveable::save();
  }

  float gx_ = 0, gy_ = 0, gz_ = -1;

 private:
  AttitudeCalibration* calibration_;
  AttitudeSensor* imu_;
  sensesp::ObservableValue<String>* status_;
  String bow_ = kBowUncalibrated;
  bool leveled_ = false;

  void apply_bow() {
    Axis bow;
    if (parse_bow(bow_.c_str(), bow)) {
      calibration_->set_bow(bow);
      ESP_LOGI("Calibration", "Bow direction set: %s", bow_.c_str());
    } else {
      calibration_->mark_uncalibrated();
      ESP_LOGI("Calibration", "Bow direction: uncalibrated (passthrough)");
    }
  }

  // Returns true only when a new level reference was actually stored. A false
  // return leaves the previous reference in place and a guiding status set.
  bool capture_level() {
    double w, x, y, z;
    if (!imu_->dmp_ready() || !imu_->latest_quat(w, x, y, z)) {
      ESP_LOGW("Calibration",
               "Level capture skipped: IMU still settling (~40 s after boot)");
      set_status("IMU still settling — wait ~40 s after boot, then Save again");
      return false;
    }
    if (!calibration_->capture_level(w, x, y, z)) {
      ESP_LOGW("Calibration",
               "Level capture skipped: the selected bow axis points up/down — "
               "pick the axis that faces the front of the boat");
      set_status("Bow axis points up/down — pick the one facing the bow");
      return false;
    }
    Vec3 g = calibration_->reference();
    gx_ = g.x;
    gy_ = g.y;
    gz_ = g.z;
    leveled_ = true;
    ESP_LOGI("Calibration",
             "Level captured: reference g_chip0 = (%.4f, %.4f, %.4f)", gx_, gy_,
             gz_);
    return true;
  }

  // Keep the persisted leveled flag equal to "compute() will use the boat
  // frame". A bow change can leave a good reference unusable (the new bow axis
  // is vertical against it) while a failed capture leaves the old flag set;
  // without this the status reads "Calibrated" while compute() falls back to
  // the chip-frame passthrough, and imu.h then publishes those raw angles as
  // vessel attitude.
  void reconcile_leveled() {
    bool usable = calibration_->frame_usable();
    leveled_ = usable;
    calibration_->set_leveled(usable);
  }

  void apply() {
    apply_bow();
    calibration_->set_reference(Vec3{gx_, gy_, gz_});
    calibration_->set_leveled(leveled_);
    reconcile_leveled();
    update_cal_status(calibration_, status_);
  }

  void set_status(const String& s) {
    if (status_) status_->set(s);
  }
};

inline const String ConfigSchema(const CalibrationConfig&) {
  String schema =
      R"JSON({"type":"object","properties":{"bow":{"title":"Bow Direction","type":"array","format":"select","uniqueItems":true,"items":{"type":"string","enum":[<<options>>]}},"gx":{"title":"Reference gx","type":"number","readOnly":true},"gy":{"title":"Reference gy","type":"number","readOnly":true},"gz":{"title":"Reference gz","type":"number","readOnly":true}}})JSON";
  std::vector<String> labels = BowLabels();
  String options;
  for (size_t i = 0; i < labels.size(); i++) {
    options += "\"" + labels[i] + "\"";
    if (i + 1 < labels.size()) options += ",";
  }
  schema.replace("<<options>>", options);
  return schema;
}

}  // namespace attitude_sensor

#endif  // ATTITUDE_SENSOR_SRC_CALIBRATION_CONFIG_H_
