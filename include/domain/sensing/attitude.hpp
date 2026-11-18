#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string_view>

#include "domain/error/effects.hpp"
#include "domain/random/counter_rng.hpp"
#include "domain/sensing/sensor.hpp"
#include "domain/spatial/transform.hpp"

namespace achilles::domain::sensing {

// An attitude estimate: the orientation an IMU's on-board fusion reports --
// a turret board's roll, pitch and yaw, say -- as Z-Y-X Euler angles of the
// sensor's axes in the world's: roll about x, pitch about y, yaw about z,
// world_from_sensor = Rz(yaw) * Ry(pitch) * Rx(roll), each in (-pi, pi]
// (pitch in [-pi/2, pi/2]). It reads its body's world pose, so what it
// models is the estimator's output, not the gyro it integrates: its error
// model is the estimate's -- a fixed misalignment (bias), heading drift
// (bias_instability, a random walk per sqrt(second)), jitter (noise_density
// or noise_sigma), resolution and latency (see AttachmentConfig::ErrorModel),
// applied per angle before wrapping.
class Attitude : public Sensor<
                     Attitude,
                     SimInputs<input::WorldTransform>,
                     domain::error::ErrorChain<domain::error::Vec3>::State,
                     domain::error::Vec3> {
 public:
  using ErrorModel = domain::error::ErrorChain<domain::error::Vec3>;
  using Pose = input::WorldTransform::Value;

  static constexpr std::string_view kListName = "attitude";

  struct Params {
    // Where the sensor sits on its body: a pose in the body's frame.
    domain::spatial::Transform<float> body_from_sensor;
    ErrorModel::Params error;
  };

  Attitude() = default;
  Attitude(AttachmentOptions options, const Params& params)
      : Sensor(options), params_(params) {}

  // Settings: period, mount (a pose in the body's frame, identity when
  // absent), and the error model's (see AttachmentConfig::ErrorModel).
  static Attitude FromConfig(AttachmentConfig& config) {
    const AttachmentOptions options = config.Options();
    return Attitude(
        options,
        Params{config.BodyFromSensor(), config.ErrorModel<domain::error::Vec3>()}
    );
  }

  // Z-Y-X Euler angles (roll, pitch, yaw) of a rotation.
  static domain::error::Vec3 EulerZyx(const math::Quaternion<float>& q) {
    const auto r = q.Normalize().ToRotationMatrix();
    const float pitch = std::asin(std::clamp(-r(2, 0), -1.0F, 1.0F));
    const float roll = std::atan2(r(2, 1), r(2, 2));
    const float yaw = std::atan2(r(1, 0), r(0, 0));
    return {roll, pitch, yaw};
  }

  void Read(
      const AttachmentTick& tick,
      const Pose& pose,
      ErrorModel::State* error,
      domain::error::Vec3* out
  ) const {
    const domain::error::Vec3 truth =
        EulerZyx((pose * params_.body_from_sensor).Rotation());
    domain::random::CounterRng rng(tick.stream, tick.tick);
    const domain::error::Vec3 measured =
        ErrorModel::Apply(params_.error, *error, rng, tick.dt, truth);
    *out = {Wrap(measured.X()), Wrap(measured.Y()), Wrap(measured.Z())};
  }

 private:
  static float Wrap(float angle) {
    constexpr float kPi = std::numbers::pi_v<float>;
    float wrapped = std::remainder(angle, 2.0F * kPi);
    if (wrapped <= -kPi) {
      wrapped += 2.0F * kPi;
    }
    return wrapped;
  }

  Params params_;
};

}  // namespace achilles::domain::sensing
