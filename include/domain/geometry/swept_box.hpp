#pragma once

#include <algorithm>
#include <cmath>
#include <concepts>

#include "domain/geometry/ray.hpp"
#include "domain/math/vector3.hpp"

namespace achilles::domain::geometry {

// The points within `radius` of the box with half-extents `extents`, in the
// shape's own frame -- the collision shape every body carries
// (algorithms/broad_phase/broad_phase_data.hpp). Scalar only, like Ray: its
// cast branches on each ray's own data.
template <std::floating_point T>
class SweptBox {
  using Vector3 = math::Vector3<T>;

 public:
  using ScalarType = T;

  constexpr SweptBox(const Vector3& extents, T radius)
      : extents_(extents), radius_(radius) {}

  // Access
  constexpr const Vector3& Extents() const { return extents_; }
  constexpr T Radius() const { return radius_; }

  // Geometry
  // Signed distance from `p` (shape coordinates) to the surface: negative
  // inside.
  T Distance(const Vector3& p) const {
    const Vector3 q = p.Abs() - extents_;
    const Vector3 outside = q.Max(Vector3::Zero());
    const T inside = std::min(std::max({q.X(), q.Y(), q.Z()}), T{0});
    return outside.Norm() + inside - radius_;
  }

  // Casts `ray` (shape coordinates) over t in [min_range, max_range]. A ray
  // that starts inside the shape -- a sensor inside its own body's collision
  // shape -- doesn't see it at all.
  //
  // The shape's bounding sphere culls almost every miss with one quadratic;
  // what's left is sphere-traced on Distance, which needs no case analysis
  // over faces, edges and corners and is exact to `tolerance` -- the same
  // trace would work for any shape with a distance function.
  RayHit<T> Cast(
      const Ray<T>& ray, T min_range, T max_range, T tolerance = T{1e-4}
  ) const {
    constexpr int kMaxSteps = 64;
    const RayHit<T> miss{max_range, false};

    // Bounding sphere: |origin + t d|^2 = R^2.
    const T bound = extents_.Norm() + radius_;
    if (bound <= T{0}) {
      return miss;
    }
    const T b = ray.Origin().Dot(ray.Direction());
    const T c = ray.Origin().SquaredNorm() - (bound * bound);
    const T discriminant = (b * b) - c;
    if (discriminant < T{0}) {
      return miss;
    }
    const T root = std::sqrt(discriminant);
    const T enter = std::max(-b - root, min_range);
    const T leave = std::min(-b + root, max_range);
    if (enter > leave) {
      return miss;
    }
    if (Distance(ray.At(min_range)) < T{0}) {
      return miss;
    }

    T t = enter;
    for (int step = 0; step < kMaxSteps && t <= leave; ++step) {
      const T d = Distance(ray.At(t));
      if (d < tolerance) {
        return RayHit<T>{t, true};
      }
      t += d;
    }
    return miss;
  }

 private:
  Vector3 extents_;
  T radius_;
};

}  // namespace achilles::domain::geometry
