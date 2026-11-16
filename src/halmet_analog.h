#ifndef HALMET_ANALOG_H_
#define HALMET_ANALOG_H_

#include <Adafruit_ADS1X15.h>

#include "sensesp/sensors/sensor.h"
#include "sensesp/transforms/transform.h"
#include "sensesp_base_app.h"

namespace halmet {

// HALMET voltage divider scale factor
const float kVoltageDividerScale = 33.3 / 3.3;

/**
 * @brief A tank's capacity in liters, set in the web UI.
 *
 * As a transform it turns the tank level (ratio 0..1) into the current volume
 * in m3 for Signal K; capacity_liters() gives the same figure to the NMEA 2000
 * fluid level sender, so one setting drives both outputs.
 */
class TankCapacity : public sensesp::Transform<float, float> {
 public:
  TankCapacity(float default_liters, const String& config_path)
      : sensesp::Transform<float, float>(config_path),
        capacity_liters_{default_liters} {
    loaded_ = this->load();
  }

  void set(const float& level) override {
    this->emit(level * capacity_liters_ / 1000.0f);  // liters -> m3
  }

  float capacity_liters() const { return capacity_liters_; }
  void set_capacity_liters(float liters) { capacity_liters_ = liters; }
  /// False until the capacity has been saved once (a fresh or upgraded unit).
  bool loaded() const { return loaded_; }

  bool to_json(JsonObject& config) override {
    config["capacity"] = capacity_liters_;
    return true;
  }

  bool from_json(const JsonObject& config) override {
    if (!config["capacity"].is<float>()) return false;
    capacity_liters_ = config["capacity"].as<float>();
    return true;
  }

 private:
  float capacity_liters_;
  bool loaded_ = false;
};

inline const String ConfigSchema(const TankCapacity& obj) {
  return R"###({
      "type": "object",
      "properties": {
        "capacity": { "title": "Capacity", "type": "number", "description": "Tank capacity in liters. Used for the Signal K volume and the NMEA 2000 fluid level message." }
      }
    })###";
}

/**
 * @brief Connect a resistive tank sender on an ADS1115 channel.
 *
 * Returns the tank level as a ratio (0..1). If `capacity` is given, it receives
 * the tank's TankCapacity so other outputs (e.g. NMEA 2000) can use the same
 * capacity setting.
 */
sensesp::FloatProducer* ConnectTankSender(Adafruit_ADS1115* ads1115,
                                          int channel, const String& name,
                                          const String& sk_id, int sort_order,
                                          bool enable_signalk_output = true,
                                          float default_capacity_liters = 120,
                                          TankCapacity** capacity = nullptr);

class ADS1115VoltageInput : public sensesp::FloatSensor {
 public:
  ADS1115VoltageInput(Adafruit_ADS1115* ads1115, int channel,
                      const String& config_path,
                      unsigned int read_interval = 500,
                      float calibration_factor = 1.0)
      : sensesp::FloatSensor(config_path),
        ads1115_{ads1115},
        channel_{channel},
        read_interval_{read_interval},
        calibration_factor_{calibration_factor} {
    load();

    repeat_event_ = set_repeat_event(read_interval_);
  }

  void update() {
    int16_t adc_output = ads1115_->readADC_SingleEnded(channel_);
    float adc_output_volts = ads1115_->computeVolts(adc_output);
    this->emit(calibration_factor_ * kVoltageDividerScale * adc_output_volts);
  }

  virtual bool to_json(JsonObject& root) override {
    root["calibration_factor"] = calibration_factor_;
    return true;
  };

  virtual bool from_json(const JsonObject& config) override {
    if (config["calibration_factor"].is<float>()) {
      calibration_factor_ = config["calibration_factor"];
      return true;
    }
    return false;
  }

 protected:
  reactesp::RepeatEvent* repeat_event_ = nullptr;

  reactesp::RepeatEvent* set_repeat_event(unsigned int read_interval) {
    if (repeat_event_ != nullptr) {
      repeat_event_->remove(sensesp::event_loop());
    }

    repeat_event_ = sensesp::event_loop()->onRepeat(
        read_interval, [this]() { this->update(); });
    return repeat_event_;
  }

 private:
  Adafruit_ADS1115* ads1115_;
  int channel_;
  unsigned int read_interval_;
  float calibration_factor_;
};

inline const String ConfigSchema(const ADS1115VoltageInput& obj) {
  const char SCHEMA[] = R"###({
      "type": "object",
      "properties": {
          "calibration_factor": { "title": "Calibration factor", "type": "number", "description": "Multiplier to apply to the raw input value" }
      }
    })###";

  return SCHEMA;
}

inline const bool ConfigRequiresRestart(const ADS1115VoltageInput& obj) {
  return true;
}

}  // namespace halmet

#endif
