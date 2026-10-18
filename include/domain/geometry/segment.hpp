#pragma once

#include <ostream>
#include <utility>
#include <xsimd/xsimd.hpp>

#include "domain/math/vector3.hpp"
#include "util/simd_ops.hpp"

namespace achilles::domain::geometry {

// The points start + t (end - start), t in [0, 1]. A capsule's inner shape,
// a box's edge, a clipped piece of either -- and, when start == end, a
// single point, which every query here still answers without a special
// case.
template <util::ArithmeticLike T>
class Segment {
  using Vector3 = math::Vector3<T>;

 public:
  using ScalarType = T;

  constexpr Segment(const Vector3& start, const Vector3& end)
      : start_(start), end_(end) {}

  // Access
  constexpr const Vector3& Start() const { return start_; }
  constexpr const Vector3& End() const { return end_; }
  constexpr Vector3 Direction() const { return end_ - start_; }
  // start + t (end - start)
  constexpr Vector3 At(const T& t) const { return start_ + Direction() * t; }

  // Geometry
  // The closest points of this segment and `other` (on this, on other).
  // s from the unclamped optimum, t from s, then s again from the clamped
  // t -- each a clamp, so no case split on which end binds; parallel
  // segments start from s = 0, and a degenerate (point) segment's own
  // parameter is simply irrelevant to where it lands.
  constexpr std::pair<Vector3, Vector3> ClosestPoints(const Segment& other
  ) const {
    constexpr float kEpsilon = 1e-12F;
    Vector3 u = Direction();
    Vector3 v = other.Direction();
    Vector3 w = start_ - other.start_;
    T uu = xsimd::max(u.Dot(u), T(kEpsilon));
    T vv = xsimd::max(v.Dot(v), T(kEpsilon));
    T uv = u.Dot(v);
    T uw = u.Dot(w);
    T vw = v.Dot(w);
    T zero(0);
    T one(1);

    // s = (uv vw - vv uw) / (uu vv - uv^2), 0 when parallel
    T den = uu * vv - uv * uv;
    T s = util::Select(
        den > T(kEpsilon) * uu * vv,
        (uv * vw - vv * uw) / xsimd::max(den, T(kEpsilon)),
        zero
    );
    s = xsimd::min(xsimd::max(s, zero), one);
    // t = clamp((uv s + vw) / vv), s = clamp((uv t - uw) / uu)
    T t = xsimd::min(xsimd::max((uv * s + vw) / vv, zero), one);
    s = xsimd::min(xsimd::max((uv * t - uw) / uu, zero), one);
    return {At(s), other.At(t)};
  }

  // Printing
  friend std::ostream& operator<<(std::ostream& os, const Segment& s) {
    os << "Segment(" << s.start_ << ", " << s.end_ << ")";
    return os;
  }

 private:
  Vector3 start_;
  Vector3 end_;
};

}  // namespace achilles::domain::geometry
