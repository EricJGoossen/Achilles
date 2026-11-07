#pragma once

#include <string_view>

#include "domain/error/effects.hpp"
#include "domain/random/counter_rng.hpp"
#include "domain/sensing/sensor.hpp"
#include "domain/spatial/transform.hpp"

namespace achilles::domain::sensing {

// A three-axis accelerometer: specific force (m/s^2) at its mount point, in
// its own axes -- the acceleration of that point minus gravity, so a sensor
// at rest reads +g straight up. ABA seeds its base with the base
// acceleration (-gravity, see SimConfig), so the body acceleration it reads
// (ABA's kSpatialAcceleration, constrained by the contact solver -- see its
// StepBodyAccelerationOp) already is specific force at the body origin.
// Carried into the sensor's frame (Params::sensor_from_body), its linear
// part plus omega x v is specific force at the sensor -- the omega x v
// term because a spatial acceleration is taken with the reference point
// fixed in space. Its error model carries bias, scale error, bias
// instability, velocity random walk (noise density), range and
// resolution.
class Accelerometer
    : public Sensor<
          Accelerometer,
          SimInputs<input::SpatialVelocity, input::SpatialAcceleration>,
          domain::error::ErrorChain<domain::error::Vec3>::State,
          domain::error::Vec3> {
 public:
  using ErrorModel = domain::error::ErrorChain<domain::error::Vec3>;
  using Velocity = input::SpatialVelocity::Value;
  using Acceleration = input::SpatialAcceleration::Value;

  static constexpr std::string_view kListName = "accelerometer";

  struct Params {
    // The inverse of where the sensor sits on its body (its `mount`, a pose
    // in the body's frame): carries the body's motion into the sensor's.
    domain::spatial::Transform<float> sensor_from_body;
    ErrorModel::Params error;
  };

  Accelerometer() = default;
  Accelerometer(AttachmentOptions options, const Params& params)
      : Sensor(options), params_(params) {}

  // Settings: period, mount (a pose in the body's frame, identity when
  // absent), and the error model's (see AttachmentConfig::ErrorModel).
  static Accelerometer FromConfig(AttachmentConfig& config) {
    const AttachmentOptions options = config.Options();
    return Accelerometer(
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
      const Acceleration& acceleration,
      ErrorModel::State* error,
      domain::error::Vec3* out
  ) const {
    const auto v = params_.sensor_from_body.Apply(velocity);
    const auto a = params_.sensor_from_body.Apply(acceleration);
    const domain::error::Vec3 specific_force =
        a.Linear() + v.Angular().Cross(v.Linear());
    domain::random::CounterRng rng(tick.stream, tick.tick);
    *out =
        ErrorModel::Apply(params_.error, *error, rng, tick.dt, specific_force);
  }

 private:
  Params params_;
};

}  // namespace achilles::domain::sensing
