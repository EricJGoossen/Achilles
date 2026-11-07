#pragma once

#include <string_view>

#include "domain/sensing/sensor.hpp"

namespace achilles::domain::sensing {

// An ideal sensor: reports one sim quantity exactly, with no error -- ground
// truth, for logging, for checking a real sensor model against, or for a
// controller that's allowed to cheat. Output is the quantity's own value (e.g.
// Transform<float> for a world transform), and it's attached in an .arow under
// the quantity's own name (its kName).
template <typename Quantity>
class TruthSensor : public Sensor<
                        TruthSensor<Quantity>,
                        SimInputs<Quantity>,
                        NoState,
                        typename Quantity::Value> {
  using Base = Sensor<
      TruthSensor<Quantity>,
      SimInputs<Quantity>,
      NoState,
      typename Quantity::Value>;

 public:
  using Value = typename Quantity::Value;
  static constexpr std::string_view kListName = Quantity::kName;

  TruthSensor() = default;
  explicit TruthSensor(AttachmentOptions options) : Base(options) {}

  // Takes only the options every sensor has.
  static TruthSensor FromConfig(AttachmentConfig& config) {
    return TruthSensor(config.Options());
  }

  void Read(
      const AttachmentTick& /*tick*/,
      const Value& truth,
      NoState* /*state*/,
      Value* out
  ) const {
    *out = truth;
  }

  // Starts from the quantity's own initial value.
  void InitializeOutput(
      const AttachmentTick& /*tick*/, const Value& truth, Value* out
  ) const {
    *out = truth;
  }
};

// The truth sensors, by what they read.
namespace truth {

using JointPosition = TruthSensor<input::JointPosition>;
using JointVelocity = TruthSensor<input::JointVelocity>;
using JointAcceleration = TruthSensor<input::JointAcceleration>;
using WorldTransform = TruthSensor<input::WorldTransform>;
using SpatialVelocity = TruthSensor<input::SpatialVelocity>;
using SpatialAcceleration = TruthSensor<input::SpatialAcceleration>;

}  // namespace truth

}  // namespace achilles::domain::sensing
