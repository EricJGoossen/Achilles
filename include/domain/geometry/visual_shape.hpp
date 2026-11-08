#pragma once

#include <cstdint>

#include "domain/math/quaternion.hpp"
#include "domain/spatial/transform.hpp"

namespace achilles::domain::geometry {

// The shape a body is seen as (its .arow `visual_shape`): drawn at its
// `visual_pose` in the body's frame, sized by its `visual_extents`
// half-extents, colored by its `visual_color`. A body whose extents are zero
// is never seen; one with no `visual_shape` is a box, and one with no
// `visual_pose` sits on its joint's frame.
enum class VisualShape : std::uint8_t {
  kBox = 0,
  kSphere = 1,
  kCylinder = 2,
};

// The shape a `visual_shape` code names, rounding to the nearest; anything
// unrecognized is a box.
inline VisualShape ShapeOf(float code) {
  if (code > 0.5F && code < 1.5F) {
    return VisualShape::kSphere;
  }
  if (code > 1.5F && code < 2.5F) {
    return VisualShape::kCylinder;
  }
  return VisualShape::kBox;
}

// A `visual_pose` as a transform: an unset (all-zero) rotation reads as
// identity, the same convention a collision pose follows.
template <typename T>
spatial::Transform<T> ToVisualPose(const spatial::Transform<T>& pose) {
  const math::Quaternion<T>& q = pose.Rotation();
  const bool is_zero =
      q.W() == T{0} && q.X() == T{0} && q.Y() == T{0} && q.Z() == T{0};
  return {
      pose.Translation(),
      math::Quaternion<T>(is_zero ? T{1} : q.W(), q.X(), q.Y(), q.Z())
  };
}

}  // namespace achilles::domain::geometry
