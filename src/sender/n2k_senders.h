// NMEA 2000 senders for attitude (PGN 127257) and rate of turn (PGN 127251).
// Each sender uses RepeatExpiring inputs: stale values are sent as N2kDoubleNA
// so the bus always sees messages at the 100 ms interval the NMEA 2000 standard
// requires. Takes CountingNMEA2000* (not tNMEA2000*) because SendMsg is not
// virtual on the library base class.

#ifndef ATTITUDE_SENSOR_SRC_SENDER_N2K_SENDERS_H_
#define ATTITUDE_SENSOR_SRC_SENDER_N2K_SENDERS_H_

#include <N2kMessages.h>
#include <NMEA2000.h>

#include <cmath>

#include "counting_nmea2000.h"
#include "sensesp/system/expiring_value.h"
#include "sensesp/system/saveable.h"
#include "sensesp/system/serializable.h"
#include "sensesp/transforms/repeat.h"

namespace attitude_sensor {

class N2kSender : public sensesp::FileSystemSaveable,
                  virtual public sensesp::Serializable {
 public:
  N2kSender(String config_path)
      : sensesp::FileSystemSaveable{config_path}, sensesp::Serializable() {}

  virtual void enable() = 0;

  void disable() {
    if (this->sender_reaction_ != nullptr) {
      sensesp::event_loop()->remove(this->sender_reaction_);
      this->sender_reaction_ = nullptr;
    }
  }

 protected:
  reactesp::RepeatReaction* sender_reaction_ = nullptr;
};

// Sends vessel attitude as NMEA 2000 PGN 127257. roll_ and pitch_ are
// RepeatExpiring inputs (sent as N2kDoubleNA when stale). Yaw is always sent as
// N2kDoubleNA because this device has no magnetic heading reference.
class N2kAttitudeSender : public N2kSender {
 public:
  N2kAttitudeSender(String config_path, CountingNMEA2000* nmea2000,
                    bool enable = true)
      : N2kSender{config_path},
        nmea2000_{nmea2000},
        repeat_interval_{100},
        expiry_{5000} {
    if (enable) {
      this->enable();
    }
  }

  void enable() override {
    if (this->sender_reaction_ == nullptr) {
      this->sender_reaction_ =
          sensesp::event_loop()->onRepeat(repeat_interval_, [this]() {
            tN2kMsg N2kMsg;
            double pitch = this->pitch_.get();
            double roll = this->roll_.get();
            SetN2kAttitude(N2kMsg, 255, N2kDoubleNA,
                           std::isnan(pitch) ? N2kDoubleNA : pitch,
                           std::isnan(roll) ? N2kDoubleNA : roll);
            this->nmea2000_->SendMsg(N2kMsg);
          });
    }
  }

  unsigned int repeat_interval_;
  unsigned int expiry_;

  sensesp::RepeatExpiring<double> roll_{repeat_interval_, expiry_};
  sensesp::RepeatExpiring<double> pitch_{repeat_interval_, expiry_};

 protected:
  CountingNMEA2000* nmea2000_;
};

// Sends rate of turn (yaw rate) as NMEA 2000 PGN 127251. rate_ is a
// RepeatExpiring input, sent as N2kDoubleNA when stale.
class N2kRateOfTurnSender : public N2kSender {
 public:
  N2kRateOfTurnSender(String config_path, CountingNMEA2000* nmea2000,
                      bool enable = true)
      : N2kSender{config_path},
        nmea2000_{nmea2000},
        repeat_interval_{100},
        expiry_{5000} {
    if (enable) {
      this->enable();
    }
  }

  void enable() override {
    if (this->sender_reaction_ == nullptr) {
      this->sender_reaction_ =
          sensesp::event_loop()->onRepeat(repeat_interval_, [this]() {
            tN2kMsg N2kMsg;
            double rate = this->rate_.get();
            SetN2kRateOfTurn(N2kMsg, 255,
                             std::isnan(rate) ? N2kDoubleNA : rate);
            this->nmea2000_->SendMsg(N2kMsg);
          });
    }
  }

  unsigned int repeat_interval_;
  unsigned int expiry_;

  sensesp::RepeatExpiring<double> rate_{repeat_interval_, expiry_};

 protected:
  CountingNMEA2000* nmea2000_;
};

}  // namespace attitude_sensor

#endif  // ATTITUDE_SENSOR_SRC_SENDER_N2K_SENDERS_H_
