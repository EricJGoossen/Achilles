#pragma once

#include <string_view>

#include "domain/error/effects.hpp"
#include "domain/random/counter_rng.hpp"
#include "domain/sensing/sensor.hpp"
#include "domain/spatial/transform.hpp"

namespace achilles::domain::sensing {

// A three-axis rate gyroscope: the body's angular velocity (rad/s), in the
// sensor's axes. Angular velocity is the same everywhere on a rigid body,
// so only the mount's rotation matters (see Params::sensor_from_body). Its
// error model carries the usual gyro terms: turn-on bias, scale error, rate
// random walk (bias instability), angle random walk (noise density), range and
// resolution.
class Gyroscope : public Sensor<
                      Gyroscope,
                      SimInputs<input::SpatialVelocity>,
                      domain::error::ErrorChain<domain::error::Vec3>::State,
                      domain::error::Vec3> {
 public:
  using ErrorModel = domain::error::ErrorChain<domain::error::Vec3>;
  using Velocity = input::SpatialVelocity::Value;

  static constexpr std::string_view kListName = "gyroscope";

  struct Params {
    // The inverse of where the sensor sits on its body (its `mount`, a pose
    // in the body's frame): carries the body's motion into the sensor's.
    domain::spatial::Transform<float> sensor_from_body;
    ErrorModel::Params error;
  };

  Gyroscope() = default;
  Gyroscope(AttachmentOptions options, const Params& params)
      : Sensor(options), params_(params) {}

  // Settings: period, mount (a pose in the body's frame, identity when
  // absent), and the error model's (see AttachmentConfig::ErrorModel).
  static Gyroscope FromConfig(AttachmentConfig& config) {
    const AttachmentOptions options = config.Options();
    return Gyroscope(
        options,
        Params{
            config.BodyFromSensor().Inverse(),
            config.ErrorModel<domain::error::Vec3>()
        }
    );
  }

  void Read(
      const AttachmentTick& tick,
      const Velocity& velocity,
      ErrorModel::State* error,
      domain::error::Vec3* out
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

}  // namespace achilles::domain::sensing
