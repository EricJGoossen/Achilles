#pragma once

#include <string_view>

#include "domain/math/matrix.hpp"
#include "domain/math/scalar.hpp"
#include "domain/math/vector3.hpp"
#include "domain/spatial/dual.hpp"
#include "domain/spatial/transform.hpp"

// The physical quantities of the sim an attachment can ask for, by what they
// are rather than who computes them: a kind lists these in its SimInputs and
// receives each one's Value. One with a kName can also be sensed exactly, by a
// truth sensor attached under that name. Which field of the running sim holds
// one is the engine side's business
// (algorithms/attachment_kinds/sim_input.hpp).
namespace achilles::domain::attachment::input {

// At the joint itself.
struct JointPosition {
  using Value = spatial::Transform<float>;
  static constexpr std::string_view kName = "joint_position";
};
struct JointVelocity {
  using Value = spatial::SpatialVelocity<float>;
  static constexpr std::string_view kName = "joint_velocity";
};
struct JointAcceleration {
  using Value = spatial::SpatialAcceleration<float>;
  static constexpr std::string_view kName = "joint_acceleration";
};
struct JointSubspace {
  using Value = math::Matrix<float, 6, 6>;
};

// The body it moves, in world.
struct WorldTransform {
  using Value = spatial::Transform<float>;
  static constexpr std::string_view kName = "world_transform";
};
struct SpatialVelocity {
  using Value = spatial::SpatialVelocity<float>;
  static constexpr std::string_view kName = "spatial_velocity";
};
struct SpatialAcceleration {
  using Value = spatial::SpatialAcceleration<float>;
  static constexpr std::string_view kName = "spatial_acceleration";
};

// The body's collision shape.
struct CollisionPose {
  using Value = spatial::Transform<float>;
};
struct CollisionExtents {
  using Value = math::Vector3<float>;
};
struct CollisionRadius {
  using Value = math::Scalar<float>;
};

}  // namespace achilles::domain::attachment::input
