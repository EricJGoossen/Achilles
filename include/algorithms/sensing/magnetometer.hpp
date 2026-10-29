#pragma once

#include <string_view>

#include "algorithms/aba/aba_step.hpp"
#include "algorithms/sensing/sensor.hpp"
#include "domain/random/counter_rng.hpp"
#include "domain/sensing/effects.hpp"
#include "domain/spatial/transform.hpp"

namespace achilles::algorithms::sensing {

// A three-axis magnetometer: a fixed world magnetic field, seen in the
// sensor's axes -- the heading reference of a 9-axis IMU. The field is a
// per-instance setting (world_field, in world axes, any unit -- gauss by
// convention; the default is a typical mid-latitude field with +x north, Z
// up). Its error model's bias is hard-iron distortion and its scale error
// soft-iron (axis-aligned), plus noise, range and resolution.
class Magnetometer
    : public Sensor<
          Magnetometer,
          SimInputs<
              SimInput<aba::ABAAlgorithm, aba::ABAField::kWorldTransform>>,
          domain::sensing::ErrorChain<domain::sensing::Vec3>::State,
          domain::sensing::Vec3> {
 public:
  using ErrorModel = domain::sensing::ErrorChain<domain::sensing::Vec3>;
  using Pose =
      InputValueT<SimInput<aba::ABAAlgorithm, aba::ABAField::kWorldTransform>>;

  static constexpr std::string_view kListName = "magnetometer";

  struct Params {
    // The inverse of where the sensor sits on its body (its `mount`, a pose
    // in the body's frame); only its rotation matters for a field.
    domain::spatial::Transform<float> sensor_from_body;
    domain::sensing::Vec3 world_field{0.2F, 0.0F, -0.45F};
    ErrorModel::Params error;
  };

  Magnetometer() = default;
  Magnetometer(SensorOptions options, const Params& params)
      : Sensor(options), params_(params) {}

  // Settings: period, mount (a pose in the body's frame, identity when
  // absent), world_field, and the error model's (see
  // SensorConfig::ErrorModel).
  static Magnetometer FromConfig(SensorConfig& config) {
    const SensorOptions options = config.Options();
    Params params;
    params.sensor_from_body = config.Pose("mount", {}).Inverse();
    params.world_field = config.Vector("world_field", params.world_field);
    params.error = config.ErrorModel<domain::sensing::Vec3>();
    return {options, params};
  }

  void Read(
      const SensorTick& tick,
      const Pose& pose,
      ErrorModel::State* error,
      domain::sensing::Vec3* out
  ) const {
    // The pose is world-from-body, so its rotation's inverse takes a world
    // vector into body axes.
    const domain::sensing::Vec3 body_field =
        pose.Rotation().Conjugate().Rotate(params_.world_field);
    domain::random::CounterRng rng(tick.stream, tick.tick);
    *out = ErrorModel::Apply(
        params_.error,
        *error,
        rng,
        tick.dt,
        params_.sensor_from_body.Rotation().Rotate(body_field)
    );
  }

 private:
  Params params_;
};

}  // namespace achilles::algorithms::sensing
