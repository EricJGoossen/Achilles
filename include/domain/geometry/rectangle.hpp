#pragma once

#include <array>
#include <cstddef>
#include <ostream>
#include <xsimd/xsimd.hpp>

#include "domain/geometry/segment.hpp"
#include "domain/math/vector3.hpp"
#include "util/simd_ops.hpp"

namespace achilles::domain::geometry {

// A flat rectangle in space: its center, its unit normal, and two in-plane
// unit axes u, v with a half-extent along each -- a box's face, which is
// what face-contact clipping works on. A zero half-extent is allowed (a
// capsule's or a sphere's degenerate "face" is a segment or a point), and
// every query still answers it without a special case.
template <util::ArithmeticLike T>
class Rectangle {
  using Vector3 = math::Vector3<T>;
  using Mask = decltype(std::declval<T>() == std::declval<T>());

 public:
  using ScalarType = T;

  // A segment's part inside this rectangle's prism, and which lanes have
  // any part inside at all.
  struct Clipped {
    Segment<T> segment;
    Mask valid;
  };

  constexpr Rectangle(
      const Vector3& center,
      const Vector3& normal,
      const Vector3& u,
      const Vector3& v,
      const T& half_u,
      const T& half_v
  )
      : center_(center),
        normal_(normal),
        u_(u),
        v_(v),
        half_u_(half_u),
        half_v_(half_v) {}

  // Per-lane choice between two rectangles
  static constexpr Rectangle Select(
      const Mask& mask, const Rectangle& if_true, const Rectangle& if_false
  ) {
    return Rectangle(
        Vector3::Select(mask, if_true.center_, if_false.center_),
        Vector3::Select(mask, if_true.normal_, if_false.normal_),
        Vector3::Select(mask, if_true.u_, if_false.u_),
        Vector3::Select(mask, if_true.v_, if_false.v_),
        util::Select(mask, if_true.half_u_, if_false.half_u_),
        util::Select(mask, if_true.half_v_, if_false.half_v_)
    );
  }

  // Access
  constexpr const Vector3& Center() const { return center_; }
  constexpr const Vector3& Normal() const { return normal_; }

  // c + u h_u s_u + v h_v s_v, corner i of 4, counter-clockwise from (+, +)
  constexpr Vector3 Corner(std::size_t i) const {
    constexpr std::array<float, 4> kSignU = {1.0F, -1.0F, -1.0F, 1.0F};
    constexpr std::array<float, 4> kSignV = {1.0F, 1.0F, -1.0F, -1.0F};
    return center_ + u_ * (half_u_ * T(kSignU[i])) +
           v_ * (half_v_ * T(kSignV[i]));
  }
  // corner i -> corner i + 1
  constexpr Segment<T> Edge(std::size_t i) const {
    return Segment<T>(Corner(i), Corner((i + 1) % 4));
  }

  // Geometry
  // (p - c) . n, signed height of p above the rectangle's plane
  constexpr T Height(const Vector3& p) const {
    return (p - center_).Dot(normal_);
  }

  // p + dir ((c - p) . n / (dir . n)), where the line through p along dir
  // meets the rectangle's plane. dir must not lie in the plane.
  constexpr Vector3 Hit(const Vector3& p, const Vector3& dir) const {
    return p + dir * ((center_ - p).Dot(normal_) / dir.Dot(normal_));
  }

  // The part of `segment` inside the rectangle's prism -- between its four
  // side planes, at any height -- by Liang-Barsky: each side pair bounds
  // the segment parameter to an interval, and the segment survives where
  // all of them and [0, 1] overlap. A segment parallel to a side pair is
  // bounded by +-1/epsilon there, i.e. kept whole when between the pair
  // and dropped when outside it.
  constexpr Clipped Clip(const Segment<T>& segment) const {
    T lo(0);
    T hi(1);
    Interval(segment, u_, half_u_, lo, hi);
    Interval(segment, v_, half_v_, lo, hi);
    return Clipped{Segment<T>(segment.At(lo), segment.At(hi)), lo <= hi};
  }

  // Printing
  friend std::ostream& operator<<(std::ostream& os, const Rectangle& r) {
    os << "Rectangle(" << r.center_ << ", " << r.normal_ << ")";
    return os;
  }

 private:
  // -h <= (p(t) - c) . axis <= h, intersected into [lo, hi]
  constexpr void Interval(
      const Segment<T>& segment,
      const Vector3& axis,
      const T& half,
      T& lo,
      T& hi
  ) const {
    // A degenerate side pair (h = 0) is only hit to rounding, so it gets
    // a little slack.
    constexpr float kSlack = 1e-6F;
    constexpr float kParallel = 1e-12F;
    T start = (segment.Start() - center_).Dot(axis);
    T rate = segment.Direction().Dot(axis);
    T safe_rate =
        util::Select(xsimd::abs(rate) > T(kParallel), rate, T(kParallel));
    T bound = half + T(kSlack);
    T t0 = (-bound - start) / safe_rate;
    T t1 = (bound - start) / safe_rate;
    lo = xsimd::max(lo, xsimd::min(t0, t1));
    hi = xsimd::min(hi, xsimd::max(t0, t1));
  }

  Vector3 center_;
  Vector3 normal_;
  Vector3 u_;
  Vector3 v_;
  T half_u_;
  T half_v_;
};

}  // namespace achilles::domain::geometry
