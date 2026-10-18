#pragma once

#include <cstddef>
#include <ostream>

#include "domain/geometry/rectangle.hpp"
#include "domain/math/matrix.hpp"
#include "domain/math/vector3.hpp"
#include "util/simd_ops.hpp"

namespace achilles::domain::geometry {

// A box in some frame: its center, its orientation R (whose columns are the
// box's own axes), and its half-extent along each of those axes. Every query
// here is branch-free whatever the box's pose -- a rotate, a per-axis clamp
// or sign, a rotate back, a select -- so a batch of boxes answers it lane
// for lane. Rᵀ is kept alongside R since nearly every query moves a point
// into the box's own frame first. Zero half-extents are allowed: a segment
// (a capsule's inner shape) or a point (a sphere's) is a box too, and every
// query still answers it.
//
// A working value an Op builds and queries, not a field type: it has no
// Assembler, and nothing stores one per row.
template <util::ArithmeticLike T>
class Box {
  using Vector3 = math::Vector3<T>;
  using Matrix3x3 = math::Matrix<T, 3, 3>;
  using Mask = decltype(std::declval<T>() == std::declval<T>());

 public:
  using ScalarType = T;

  constexpr Box(
      const Vector3& center,
      const Matrix3x3& rotation,
      const Vector3& half_extents
  )
      : center_(center),
        rotation_(rotation),
        inverse_(rotation.Transpose()),
        half_extents_(half_extents) {}

  // Per-lane choice between two boxes
  static constexpr Box Select(
      const Mask& mask, const Box& if_true, const Box& if_false
  ) {
    return Box(
        Vector3::Select(mask, if_true.center_, if_false.center_),
        Matrix3x3::Select(mask, if_true.rotation_, if_false.rotation_),
        Vector3::Select(mask, if_true.half_extents_, if_false.half_extents_)
    );
  }

  // Access
  constexpr const Vector3& Center() const { return center_; }
  constexpr const Matrix3x3& Rotation() const { return rotation_; }
  constexpr const Vector3& HalfExtents() const { return half_extents_; }
  // R_i, the box's i-th axis
  constexpr Vector3 Axis(std::size_t i) const {
    return Vector3(rotation_.Column(i));
  }
  // h_i, its half-extent along R_i
  constexpr T HalfExtent(std::size_t i) const { return half_extents_[i]; }

  // Geometry
  // c + R clamp(Rᵀ (x - c), -h, h), the box's closest point to x
  constexpr Vector3 Project(const Vector3& x) const {
    Vector3 local = inverse_ * (x - center_);
    return center_ + rotation_ * local.Clamp(-half_extents_, half_extents_);
  }
  // c + sum_k R_k h_k sgn(R_k . dir), the box's furthest point along dir
  constexpr Vector3 Support(const Vector3& dir) const {
    Vector3 out = center_;
    for (std::size_t k = 0; k < 3; ++k) {
      Vector3 axis = Axis(k);
      out += axis * (HalfExtent(k) * util::Sign(axis.Dot(dir)));
    }
    return out;
  }
  // c + sum_k R_k h_k s_k, s_k = +-1 from bit k of `corner` (0..7)
  constexpr Vector3 Vertex(std::size_t corner) const {
    Vector3 out = center_;
    for (std::size_t k = 0; k < 3; ++k) {
      T side(static_cast<float>(((corner >> k) & 1U) * 2U) - 1.0F);
      out += Axis(k) * (HalfExtent(k) * side);
    }
    return out;
  }
  // h . |Rᵀ L|, the box's half-width along L
  constexpr T HalfWidth(const Vector3& axis) const {
    return half_extents_.Dot((inverse_ * axis).Abs());
  }
  // The face whose outward normal is most aligned with dir: axis k with the
  // largest |R_k . dir|, on the side sgn(R_k . dir), its in-plane axes the
  // other two -- picked per lane by select among the three axes.
  constexpr Rectangle<T> FaceAlong(const Vector3& dir) const {
    T a0 = xsimd::abs(Axis(0).Dot(dir));
    T a1 = xsimd::abs(Axis(1).Dot(dir));
    T a2 = xsimd::abs(Axis(2).Dot(dir));
    Mask pick0 = (a0 >= a1) & (a0 >= a2);
    Mask pick1 = !pick0 & (a1 >= a2);

    Rectangle<T> face0 = Face(0, dir);
    Rectangle<T> face1 = Face(1, dir);
    Rectangle<T> face2 = Face(2, dir);
    return Rectangle<T>::Select(
        pick0, face0, Rectangle<T>::Select(pick1, face1, face2)
    );
  }

  // Printing
  friend std::ostream& operator<<(std::ostream& os, const Box& b) {
    os << "Box(" << b.center_ << ", " << b.half_extents_ << ")";
    return os;
  }

 private:
  // The face along +-R_k, on dir's side, spanned by the other two axes
  constexpr Rectangle<T> Face(std::size_t k, const Vector3& dir) const {
    std::size_t i = (k + 1) % 3;
    std::size_t j = (k + 2) % 3;
    Vector3 normal = Axis(k) * util::Sign(Axis(k).Dot(dir));
    return Rectangle<T>(
        center_ + normal * HalfExtent(k),
        normal,
        Axis(i),
        Axis(j),
        HalfExtent(i),
        HalfExtent(j)
    );
  }

  Vector3 center_;
  Matrix3x3 rotation_;
  Matrix3x3 inverse_;
  Vector3 half_extents_;
};

}  // namespace achilles::domain::geometry
