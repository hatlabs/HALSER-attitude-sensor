// HALSER Attitude Sensor Firmware — application entry point.
// Wires the data pipeline: ICM-20948 (I2C, DMP game rotation vector) →
// calibrated roll/pitch/rates → NMEA 2000 (PGN 127257, 127251) + Signal K.
// The web UI provides a single calibration card: mounting orientation (bow
// direction) and level reference capture on Save.

#include <NMEA2000_esp32.h>

#include <memory>

#include "Wire.h"
#include "attitude_calibration.h"
#include "calibration_config.h"
#include "counting_nmea2000.h"
#include "elapsedMillis.h"
#include "imu.h"
#include "sender/n2k_senders.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_types.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/serial_number.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/types/position.h"
#include "sensesp/ui/config_item.h"
#include "sensesp/ui/status_page_item.h"
#include "sensesp/ui/ui_controls.h"
#include "sensesp_app_builder.h"

using namespace sensesp;
using namespace attitude_sensor;

// HALSER pin assignments
constexpr gpio_num_t kCANTxPin = GPIO_NUM_4;
constexpr gpio_num_t kCANRxPin = GPIO_NUM_5;
constexpr int kI2CSDAPin = 6;
constexpr int kI2CSCLPin = 7;
constexpr int kButtonPin = 9;

// ICM-20948 I2C AD0 level: 1 → address 0x69, 0 → address 0x68.
// SparkFun breakouts default to AD0 high (0x69).
constexpr uint8_t kIMUAd0 = 1;
// IMU sampling interval (ms). 100 ms feeds the 100 ms NMEA 2000 attitude/ROT
// PGNs and the matching Signal K report rate below.
constexpr unsigned int kIMUIntervalMs = 100;
constexpr unsigned int kSKReportIntervalMs = 100;

ObservableValue<int> n2k_rx_counter = 0;

elapsedMillis n2k_time_since_rx = 0;

void setup() {
  Serial.setTxTimeoutMs(0);
  SetupLogging(ESP_LOG_DEBUG);

  Wire.setPins(kI2CSDAPin, kI2CSCLPin);
  Wire.begin();

  // SensESP application
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    ->set_hostname("attitude")
                    ->set_button_pin(kButtonPin)
                    ->enable_ota("change-me")
                    ->get_app();

  /////////////////////////////////////////////////////////////////////
  // Initialize NMEA 2000 functionality

  CountingNMEA2000* nmea2000 = new CountingNMEA2000(kCANTxPin, kCANRxPin);

  // 64-frame CAN buffers: enough to absorb a fast-packet burst (up to 32 frames
  // each) while keeping the static footprint modest on the memory-constrained C3.
  nmea2000->SetN2kCANSendFrameBufSize(64);
  nmea2000->SetN2kCANReceiveFrameBufSize(64);

  nmea2000->SetProductInformation(
      "20260914",      // Manufacturer's Model serial code (max 32 chars)
      120,             // Manufacturer's product code
      "Attitude-N2K",  // Manufacturer's Model ID (max 33 chars)
      "1.0.0",         // Manufacturer's Software version code (max 40 chars)
      "1.0.0"          // Manufacturer's Model version (max 24 chars)
  );

  nmea2000->SetDeviceInformation(
      GetBoardSerialNumber(),  // Unique number
      140,                     // Device function: Attitude (heading/pitch/roll)
      60,                      // Device class: Navigation
      2046);                   // Manufacturer code

  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, 75);
  nmea2000->SetMsgHandler([](const tN2kMsg& msg) {
    n2k_rx_counter = n2k_rx_counter.get() + 1;
    n2k_time_since_rx = 0;
  });
  nmea2000->EnableForward(false);
  nmea2000->Open();

  event_loop()->onRepeat(1, [nmea2000]() { nmea2000->ParseMessages(); });

  /////////////////////////////////////////////////////////////////////
  // ICM-20948 IMU: attitude (roll/pitch) and attitude rates

  // Shared calibration state. Default is uncalibrated (passthrough); the config
  // objects below load persisted state into it and mutate it.
  auto calibration = std::make_shared<AttitudeCalibration>();

  auto imu = std::make_shared<AttitudeSensor>(&Wire, calibration.get(), kIMUAd0,
                                              kIMUIntervalMs);

  /////////////////////////////////////////////////////////////////////
  // NMEA 2000 attitude and rate-of-turn senders

  auto attitude_n2k = std::make_shared<N2kAttitudeSender>(
      "/Attitude/NMEA2000", nmea2000, true);
  imu->roll.connect_to(&(attitude_n2k->roll_));
  imu->pitch.connect_to(&(attitude_n2k->pitch_));

  auto rot_n2k = std::make_shared<N2kRateOfTurnSender>("/Rate of Turn/NMEA2000",
                                                       nmea2000, true);
  imu->yaw_rate.connect_to(&(rot_n2k->rate_));

  /////////////////////////////////////////////////////////////////////
  // Signal K outputs

  // Attitude → Signal K navigation.attitude.
  // Yaw is left as NAN (serialized as null): no magnetic heading reference.
  auto attitude_sensor_sk = std::make_shared<RepeatSensor<AttitudeVector>>(
      kSKReportIntervalMs, [imu]() {
        return AttitudeVector(imu->roll.get(), imu->pitch.get(), NAN);
      });
  auto attitude_sk = std::make_shared<SKOutput<AttitudeVector>>(
      "navigation.attitude", "/SK Path/Attitude",
      new SKMetadata("rad", "Vessel Attitude"));
  attitude_sensor_sk->connect_to(attitude_sk);

  // Yaw rate → Signal K navigation.rateOfTurn.
  auto yaw_rate_sensor = std::make_shared<RepeatSensor<float>>(
      kSKReportIntervalMs, [imu]() { return imu->yaw_rate.get(); });
  auto yaw_rate_sk = std::make_shared<SKOutputFloat>(
      "navigation.rateOfTurn", "/SK Path/Rate of Turn",
      new SKMetadata("rad/s", "Rate of Turn"));
  yaw_rate_sensor->connect_to(yaw_rate_sk);

  // Roll/pitch rates: no standard Signal K path or NMEA 2000 PGN exists for
  // these. Uncomment and assign appropriate custom paths if needed:
  //
  // auto roll_rate_sensor = std::make_shared<RepeatSensor<float>>(
  //     kSKReportIntervalMs, [imu]() { return imu->roll_rate.get(); });
  // auto roll_rate_sk = std::make_shared<SKOutputFloat>(
  //     "navigation.attitude.rateOfRoll", "/SK Path/Roll Rate",
  //     new SKMetadata("rad/s", "Rate of Roll"));
  // roll_rate_sensor->connect_to(roll_rate_sk);
  //
  // auto pitch_rate_sensor = std::make_shared<RepeatSensor<float>>(
  //     kSKReportIntervalMs, [imu]() { return imu->pitch_rate.get(); });
  // auto pitch_rate_sk = std::make_shared<SKOutputFloat>(
  //     "navigation.attitude.rateOfPitch", "/SK Path/Pitch Rate",
  //     new SKMetadata("rad/s", "Rate of Pitch"));
  // pitch_rate_sensor->connect_to(pitch_rate_sk);

  /////////////////////////////////////////////////////////////////////
  // IMU calibration (web UI)

  auto cal_status =
      std::make_shared<ObservableValue<String>>(String("Uncalibrated"));

  auto calibration_config = std::make_shared<CalibrationConfig>(
      calibration.get(), imu.get(), cal_status.get(), "/IMU/Calibration");
  ConfigItem(calibration_config)
      ->set_title("Attitude Calibration")
      ->set_description(
          "Set the bow direction, then — with the boat at rest on an even keel — "
          "Save to calibrate the level attitude (zero heel and trim). Bow "
          "direction is which module axis points toward the bow; module axes "
          "(accelerometer frame): +X = east, -X = west; +Y = north, -Y = south; "
          "+Z = up, -Z = down. Pick the closest axis — the exact mounting tilt "
          "is absorbed by the level capture.")
      ->set_sort_order(600);

  auto cal_status_item = std::make_shared<StatusPageItem<String>>(
      "IMU Calibration", cal_status->get(), "IMU", 600);
  cal_status->connect_to(cal_status_item);

  auto rad_to_deg = [](float rad) { return rad * 180.0f / (float)M_PI; };
  auto roll_status =
      std::make_shared<StatusPageItem<float>>("Roll (deg)", NAN, "IMU", 610);
  imu->roll.connect_to(std::make_shared<LambdaTransform<float, float>>(rad_to_deg))
      ->connect_to(roll_status);
  auto pitch_status =
      std::make_shared<StatusPageItem<float>>("Pitch (deg)", NAN, "IMU", 620);
  imu->pitch
      .connect_to(std::make_shared<LambdaTransform<float, float>>(rad_to_deg))
      ->connect_to(pitch_status);

  /////////////////////////////////////////////////////////////////////
  // Configuration elements

  auto enable_n2k_watchdog_config = std::make_shared<CheckboxConfig>(
      false, "Enable NMEA 2000 Watchdog", "/NMEA2000/Enable Watchdog");

  ConfigItem(enable_n2k_watchdog_config)
      ->set_title("Enable NMEA 2000 Watchdog")
      ->set_description(
          "Enable the NMEA 2000 watchdog. If enabled, the device will reboot "
          "after two minutes if no NMEA 2000 messages are received. This "
          "setting requires a device restart to take effect.")
      ->set_requires_restart(true)
      ->set_sort_order(100);

  if (enable_n2k_watchdog_config->get_value()) {
    event_loop()->onRepeat(1000, [nmea2000]() {
      if (n2k_time_since_rx > 120000) {
        ESP_LOGE("NMEA2000", "No messages received in 2 minutes. Restarting.");
        delay(10);
        ESP.restart();
      }
    });
  }

  /////////////////////////////////////////////////////////////////////
  // Status page items

  auto n2k_rx_ui_output = std::make_shared<StatusPageItem<int>>(
      "NMEA 2000 Received Messages", 0, "NMEA 2000", 300);
  n2k_rx_counter.connect_to(n2k_rx_ui_output);

  auto n2k_tx_ui_output = std::make_shared<StatusPageItem<int>>(
      "NMEA 2000 Transmitted Messages", 0, "NMEA 2000", 310);
  nmea2000->tx_count_.connect_to(n2k_tx_ui_output);

  // Largest contiguous free block. This, not total free memory, gates large
  // allocations like the ~40 KB TLS handshake, so surface it alongside free
  // memory on the status page.
  auto largest_block_status = std::make_shared<StatusPageItem<int>>(
      "Largest free block (bytes)", 0, "System", 250);
  event_loop()->onRepeat(2000, [largest_block_status]() {
    largest_block_status->set(static_cast<int>(ESP.getMaxAllocHeap()));
  });

  // Main-loop task stack headroom. uxTaskGetStackHighWaterMark returns the
  // running task's minimum free stack in bytes on ESP-IDF.
  auto main_loop_stack_status = std::make_shared<StatusPageItem<int>>(
      "Main loop min free stack (bytes)", 0, "System", 260);
  event_loop()->onRepeat(2000, [main_loop_stack_status]() {
    main_loop_stack_status->set(
        static_cast<int>(uxTaskGetStackHighWaterMark(nullptr)));
  });

  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }
