#pragma once

#include <concepts>

#include "domain/math/vector3.hpp"

namespace achilles::domain::geometry {

// The half-line origin + t direction, t >= 0, `direction` a unit vector.
// What a range sensor's beam is cast along. Scalar only (T a floating-point
// type, never a batch): every query on a ray branches on that ray's own
// data.
template <std::floating_point T>
class Ray {
  using Vector3 = math::Vector3<T>;

 public:
  using ScalarType = T;

  constexpr Ray(const Vector3& origin, const Vector3& direction)
      : origin_(origin), direction_(direction) {}

  // Access
  constexpr const Vector3& Origin() const { return origin_; }
  constexpr const Vector3& Direction() const { return direction_; }
  // origin + t direction
  constexpr Vector3 At(T t) const { return origin_ + (direction_ * t); }

 private:
  Vector3 origin_;
  Vector3 direction_;
};

// What a ray cast reports: the distance along the ray to the first hit, or
// the far end of the range searched when nothing is hit within it.
template <std::floating_point T>
struct RayHit {
  T distance;
  bool hit;
};

}  // namespace achilles::domain::geometry
