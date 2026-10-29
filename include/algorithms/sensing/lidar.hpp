#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string_view>

#include "algorithms/aba/aba_step.hpp"
#include "algorithms/broad_phase/broad_phase_step.hpp"
#include "algorithms/sensing/sensor.hpp"
#include "domain/geometry/ray.hpp"
#include "domain/geometry/swept_box.hpp"
#include "domain/math/quaternion.hpp"
#include "domain/math/vector3.hpp"
#include "domain/random/counter_rng.hpp"
#include "domain/spatial/transform.hpp"

namespace achilles::algorithms::sensing {

// How many beams one lidar can cast: the fixed capacity that keeps its
// records a fixed size. How many it actually casts is a setting, up to this.
inline constexpr std::size_t kLidarMaxBeams = 1024;

// One scan: a range per beam (meters), +infinity where a beam found nothing
// within the lidar's range. Beam i is channel i / horizontal_samples,
// azimuth step i % horizontal_samples.
struct LidarScan {
  std::array<float, kLidarMaxBeams> ranges{};
  std::uint32_t beams = 0;
};

// The nearest hit so far this sample, per beam: Observe narrows it body by
// body, Read turns it into the scan and starts it over.
struct LidarReturns {
  std::array<float, kLidarMaxBeams> nearest{};
};

// A body's world pose (ABA's), its collision shape's pose within it, and
// the shape itself (the broad phase's): what a lidar reads at its own body
// and scans at every body.
using LidarPose = SimInput<aba::ABAAlgorithm, aba::ABAField::kWorldTransform>;
using LidarShapePose = SimInput<
    broad_phase::BroadPhaseAlgorithm,
    broad_phase::BroadPhaseField::kCollisionPose>;
using LidarShapeExtents = SimInput<
    broad_phase::BroadPhaseAlgorithm,
    broad_phase::BroadPhaseField::kCollisionExtents>;
using LidarShapeRadius = SimInput<
    broad_phase::BroadPhaseAlgorithm,
    broad_phase::BroadPhaseField::kCollisionRadius>;

// A scanning lidar: casts a fan (or, with several channels, a stack of
// fans) of beams from its mount against every body's collision shape, and
// reports the range to the nearest hit per beam. Each sample, Observe runs
// once per body -- placing its shape in world, culling it with one compare
// when its bounding sphere is out of range, otherwise casting every beam
// against it -- and Read applies range bias, Gaussian range noise and
// resolution. A beam starting inside a body (the lidar's own housing)
// doesn't see that body. Set a period (e.g. 0.1 s for 10 Hz): a lidar
// costs nothing on the ticks it isn't sampling.
class Lidar
    : public Sensor<Lidar, SimInputs<LidarPose>, LidarReturns, LidarScan> {
 public:
  using Scene = SceneInputs<
      LidarPose,
      LidarShapePose,
      LidarShapeExtents,
      LidarShapeRadius>;
  using Pose = InputValueT<LidarPose>;
  using Extents = InputValueT<LidarShapeExtents>;
  using Radius = InputValueT<LidarShapeRadius>;

  static constexpr std::string_view kListName = "lidar";

  struct Params {
    // Where the lidar sits on its body: a pose in the body's frame. Sensor
    // axes: x forward, y left, z up.
    domain::spatial::Transform<float> body_from_sensor;
    // Beams per channel, spread evenly across horizontal_fov (radians,
    // centered on +x).
    std::uint32_t horizontal_samples = 360;
    // Channels, spread evenly across vertical_fov (radians, centered on the
    // horizontal plane); one channel is a planar scanner.
    std::uint32_t channels = 1;
    float horizontal_fov = 2.0F * std::numbers::pi_v<float>;
    float vertical_fov = 0.0F;
    float min_range = 0.05F;
    float max_range = 30.0F;
    // Range errors, in meters: a constant offset, the standard deviation of
    // per-beam Gaussian noise, and the reported resolution (0: continuous).
    float range_bias = 0.0F;
    float range_noise = 0.0F;
    float resolution = 0.0F;
  };

  Lidar() = default;
  // Throws std::invalid_argument for no beams, more than kLidarMaxBeams, or
  // an empty range interval.
  Lidar(SensorOptions options, const Params& params)
      : Sensor(options), params_(params) {
    const std::uint64_t beams =
        std::uint64_t{params_.horizontal_samples} * params_.channels;
    if (beams == 0 || beams > kLidarMaxBeams) {
      throw std::invalid_argument(
          "Lidar: horizontal_samples * channels must be in [1, "
          "kLidarMaxBeams]."
      );
    }
    if (!(params_.min_range >= 0.0F && params_.max_range > params_.min_range)) {
      throw std::invalid_argument("Lidar: needs 0 <= min_range < max_range.");
    }
    AimBeams();
  }

  // Settings: period, mount (a pose in the body's frame, identity when
  // absent), horizontal_samples, channels, horizontal_fov, vertical_fov,
  // min_range, max_range, range_bias, range_noise, resolution.
  static Lidar FromConfig(SensorConfig& config) {
    const SensorOptions options = config.Options();
    Params params;
    params.body_from_sensor = config.Pose("mount", {});
    params.horizontal_samples =
        config.Count("horizontal_samples", params.horizontal_samples);
    params.channels = config.Count("channels", params.channels);
    params.horizontal_fov =
        config.Scalar("horizontal_fov", params.horizontal_fov);
    params.vertical_fov = config.Scalar("vertical_fov", params.vertical_fov);
    params.min_range = config.Scalar("min_range", params.min_range);
    params.max_range = config.Scalar("max_range", params.max_range);
    params.range_bias = config.Scalar("range_bias", params.range_bias);
    params.range_noise = config.Scalar("range_noise", params.range_noise);
    params.resolution = config.Scalar("resolution", params.resolution);
    return Lidar(options, params);
  }

  std::uint32_t BeamCount() const {
    return params_.horizontal_samples * params_.channels;
  }

  void Initialize(const SensorTick& /*tick*/, LidarReturns* returns) const {
    returns->nearest.fill(params_.max_range);
  }

  void Observe(
      const SensorTick& /*tick*/,
      const Pose& pose,
      const Pose& body_pose,
      const Pose& shape_pose,
      const Extents& extents,
      const Radius& radius,
      LidarReturns* returns
  ) const {
    const domain::geometry::SweptBox<float> shape(extents, radius.Value());
    const float bound = extents.Norm() + radius.Value();
    if (bound <= 0.0F) {
      return;
    }
    // The lidar's frame in the shape's: an unset (zero) shape rotation
    // reads as identity, as the broad phase reads it.
    const domain::math::Quaternion<float>& q = shape_pose.Rotation();
    const domain::spatial::Transform<float> body_from_shape(
        shape_pose.Translation(),
        q.SquaredNorm() == 0.0F ? domain::math::Quaternion<float>() : q
    );
    const domain::spatial::Transform<float> shape_from_sensor =
        (body_pose * body_from_shape).Inverse() *
        (pose * params_.body_from_sensor);
    const domain::math::Vector3<float> origin = shape_from_sensor.Translation();
    if (origin.Norm() - bound > params_.max_range) {
      return;
    }

    const std::uint32_t count = BeamCount();
    for (std::uint32_t i = 0; i < count; ++i) {
      const domain::geometry::RayHit<float> hit = shape.Cast(
          domain::geometry::Ray<float>(
              origin, shape_from_sensor.Rotation().Rotate(directions_[i])
          ),
          params_.min_range,
          returns->nearest[i]
      );
      if (hit.hit) {
        returns->nearest[i] = hit.distance;
      }
    }
  }

  void Read(
      const SensorTick& tick,
      const Pose& /*pose*/,
      LidarReturns* returns,
      LidarScan* out
  ) const {
    domain::random::CounterRng rng(tick.stream, tick.tick);
    const std::uint32_t count = BeamCount();
    out->beams = count;
    for (std::uint32_t i = 0; i < count; ++i) {
      float range = returns->nearest[i];
      returns->nearest[i] = params_.max_range;
      if (range >= params_.max_range) {
        out->ranges[i] = std::numeric_limits<float>::infinity();
        continue;
      }
      range += params_.range_bias + (params_.range_noise * rng.Normal());
      if (params_.resolution > 0.0F) {
        range = std::round(range / params_.resolution) * params_.resolution;
      }
      out->ranges[i] = std::clamp(range, params_.min_range, params_.max_range);
    }
  }

 private:
  // Each beam's direction in the sensor's own axes -- fixed per instance,
  // so worked out once, with the rest of its configuration.
  void AimBeams() {
    const std::uint32_t count = BeamCount();
    const float azimuth_step =
        params_.horizontal_fov / static_cast<float>(params_.horizontal_samples);
    const float elevation_step =
        params_.channels > 1
            ? params_.vertical_fov / static_cast<float>(params_.channels - 1)
            : 0.0F;
    for (std::uint32_t i = 0; i < count; ++i) {
      const std::uint32_t channel = i / params_.horizontal_samples;
      const std::uint32_t step = i % params_.horizontal_samples;
      const float azimuth = (-0.5F * params_.horizontal_fov) +
                            ((static_cast<float>(step) + 0.5F) * azimuth_step);
      const float elevation =
          params_.channels > 1
              ? (-0.5F * params_.vertical_fov) +
                    (static_cast<float>(channel) * elevation_step)
              : 0.0F;
      directions_[i] = domain::math::Vector3<float>(
          std::cos(elevation) * std::cos(azimuth),
          std::cos(elevation) * std::sin(azimuth),
          std::sin(elevation)
      );
    }
  }

  Params params_;
  std::array<domain::math::Vector3<float>, kLidarMaxBeams> directions_{};
};

}  // namespace achilles::algorithms::sensing
