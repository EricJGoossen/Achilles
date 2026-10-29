#pragma once

#include <string_view>

#include "algorithms/aba/aba_step.hpp"
#include "algorithms/sensing/sensor.hpp"
#include "domain/random/counter_rng.hpp"
#include "domain/sensing/effects.hpp"
#include "domain/spatial/transform.hpp"

namespace achilles::algorithms::sensing {

// A three-axis rate gyroscope: the body's angular velocity (rad/s), in the
// sensor's axes. Angular velocity is the same everywhere on a rigid body,
// so only the mount's rotation matters (see Params::sensor_from_body). Its
// error model carries the usual gyro terms: turn-on bias, scale error, rate
// random walk (bias instability), angle random walk (noise density), range and
// resolution.
class Gyroscope
    : public Sensor<
          Gyroscope,
          SimInputs<
              SimInput<aba::ABAAlgorithm, aba::ABAField::kSpatialVelocity>>,
          domain::sensing::ErrorChain<domain::sensing::Vec3>::State,
          domain::sensing::Vec3> {
 public:
  using ErrorModel = domain::sensing::ErrorChain<domain::sensing::Vec3>;
  using Velocity =
      InputValueT<SimInput<aba::ABAAlgorithm, aba::ABAField::kSpatialVelocity>>;

  static constexpr std::string_view kListName = "gyroscope";

  struct Params {
    // The inverse of where the sensor sits on its body (its `mount`, a pose
    // in the body's frame): carries the body's motion into the sensor's.
    domain::spatial::Transform<float> sensor_from_body;
    ErrorModel::Params error;
  };

  Gyroscope() = default;
  Gyroscope(SensorOptions options, const Params& params)
      : Sensor(options), params_(params) {}

  // Settings: period, mount (a pose in the body's frame, identity when
  // absent), and the error model's (see SensorConfig::ErrorModel).
  static Gyroscope FromConfig(SensorConfig& config) {
    const SensorOptions options = config.Options();
    return Gyroscope(
        options,
        Params{
            config.Pose("mount", {}).Inverse(),
            config.ErrorModel<domain::sensing::Vec3>()
        }
    );
  }

  void Read(
      const SensorTick& tick,
      const Velocity& velocity,
      ErrorModel::State* error,
      domain::sensing::Vec3* out
  ) const {
    domain::random::CounterRng rng(tick.stream, tick.tick);
    *out = ErrorModel::Apply(
        params_.error,
        *error,
        rng,
        tick.dt,
        params_.sensor_from_body.Apply(velocity).Angular()
    );
  }

 private:
  Params params_;
};

}  // namespace achilles::algorithms::sensing
