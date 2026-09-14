#ifndef ATTITUDE_SENSOR_SRC_RATE_FILTER_H_
#define ATTITUDE_SENSOR_SRC_RATE_FILTER_H_

#include <cmath>

namespace attitude_sensor {

// Exponential smoothing for differentiated rates. Reseeds from the fresh sample
// when the previous value is non-finite, so a NaN emitted during an output gap
// (e.g. while uncalibrated) does not propagate once output resumes.
inline float seed_safe_smooth(float prev, float fresh, float alpha) {
  return std::isfinite(prev) ? alpha * fresh + (1 - alpha) * prev : fresh;
}

}  // namespace attitude_sensor

#endif  // ATTITUDE_SENSOR_SRC_RATE_FILTER_H_
