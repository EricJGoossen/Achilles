#pragma once

#include <cstddef>
#include <ostream>

#include "matrix.hpp"
#include "util/simd_ops.hpp"
#include "vector3.hpp"

namespace achilles::domain::math {

// A box in some frame: its center, its orientation R (whose columns are the
// box's own axes), and its half-extent along each of those axes. Every query
// here is branch-free whatever the box's pose -- a rotate, a per-axis clamp
// or sign, a rotate back -- so a batch of boxes answers it lane for lane.
// Rᵀ is kept alongside R since nearly every query moves a point into the
// box's own frame first.
//
// A working value an Op builds and queries, not a field type: it has no
// Assembler, and nothing stores one per row.
template <util::ArithmeticLike T>
class OrientedBox {
 public:
  using ScalarType = T;

  constexpr OrientedBox(
      const Vector3<T>& center,
      const Matrix<T, 3, 3>& rotation,
      const Vector3<T>& half_extents
  )
      : center_(center),
        rotation_(rotation),
        inverse_(rotation.Transpose()),
        half_extents_(half_extents) {}

  // Access
  constexpr const Vector3<T>& Center() const { return center_; }
  constexpr const Matrix<T, 3, 3>& Rotation() const { return rotation_; }
  constexpr const Vector3<T>& HalfExtents() const { return half_extents_; }
  // R_i, the box's i-th axis
  constexpr Vector3<T> Axis(std::size_t i) const {
    return Vector3<T>(rotation_.Column(i));
  }
  // h_i, its half-extent along R_i
  constexpr T HalfExtent(std::size_t i) const { return half_extents_[i]; }

  // Geometry
  // c + R clamp(Rᵀ (x - c), -h, h), the box's closest point to x
  constexpr Vector3<T> Project(const Vector3<T>& x) const {
    Vector3<T> local = inverse_ * (x - center_);
    return center_ + rotation_ * local.Clamp(-half_extents_, half_extents_);
  }
  // c + sum_k R_k h_k sgn(R_k . dir), the box's furthest point along dir
  constexpr Vector3<T> Support(const Vector3<T>& dir) const {
    Vector3<T> out = center_;
    for (std::size_t k = 0; k < 3; ++k) {
      Vector3<T> axis = Axis(k);
      out += axis * (HalfExtent(k) * util::Sign(axis.Dot(dir)));
    }
    return out;
  }
  // c + sum_k R_k h_k s_k, s_k = +-1 from bit k of `corner` (0..7)
  constexpr Vector3<T> Vertex(std::size_t corner) const {
    Vector3<T> out = center_;
    for (std::size_t k = 0; k < 3; ++k) {
      T side(static_cast<float>(((corner >> k) & 1U) * 2U) - 1.0F);
      out += Axis(k) * (HalfExtent(k) * side);
    }
    return out;
  }
  // h . |Rᵀ L|, the box's half-width along L
  constexpr T HalfWidth(const Vector3<T>& axis) const {
    return half_extents_.Dot((inverse_ * axis).Abs());
  }

  // Printing
  friend std::ostream& operator<<(std::ostream& os, const OrientedBox& b) {
    os << "OrientedBox(" << b.center_ << ", " << b.half_extents_ << ")";
    return os;
  }

 private:
  Vector3<T> center_;
  Matrix<T, 3, 3> rotation_;
  Matrix<T, 3, 3> inverse_;
  Vector3<T> half_extents_;
};

}  // namespace achilles::domain::math
