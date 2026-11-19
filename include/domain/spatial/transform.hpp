#pragma once

#include <cstddef>

#include "domain/math/quaternion.hpp"
#include "domain/math/vector3.hpp"
#include "dual.hpp"
#include "engine/assembler.hpp"
#include "inertia.hpp"
#include "util/simd_ops.hpp"

namespace achilles::domain::spatial {

template <util::ArithmeticLike T>
class Transform {
  using Mask = decltype(std::declval<T>() == std::declval<T>());

  using Vector3 = math::Vector3<T>;
  using Quaternion = math::Quaternion<T>;

 public:
  using ScalarType = T;

  constexpr Transform() : translation_(), rotation_() {}
  constexpr Transform(const Vector3& translation, const Quaternion& rotation)
      : translation_(translation), rotation_(rotation) {}
  constexpr Transform(T tx, T ty, T tz, T qw, T qx, T qy, T qz)
      : translation_(tx, ty, tz), rotation_(qw, qx, qy, qz) {}

  static constexpr Transform<T> Identity() {
    return {Vector3::Zero(), Quaternion::Identity()};
  }
  constexpr Transform<T>& SetIdentity() {
    *this = Identity();
    return *this;
  }
  static constexpr Transform<T> Exp(const SpatialVelocity<T>& v) {
    return {v.Linear(), Quaternion::Exp(v.Angular())};
  }
  static constexpr Transform<T> PaddingSeed() { return Identity(); }

  // Exp's inverse: the rotation's rotation vector and the translation as
  // is, (angular, linear) -- so Exp(Log(x)) == x, and a joint's coordinate
  // along a subspace column s is s . Log(x).
  constexpr SpatialPosition<T> Log() const {
    return {rotation_.Log(), translation_};
  }

  // Access
  constexpr const Vector3& Translation() const { return translation_; }
  constexpr const Quaternion& Rotation() const { return rotation_; }

  // Renormalizes just the rotation component in place. Composing two exact
  // unit quaternions (operator*, or the result of Exp) is a unit
  // quaternion in exact arithmetic, but repeated composition -- the same
  // Transform accumulating position updates tick after tick, e.g.
  // IntegratePositionOp's own `x_joint_out *= Exp(...)` -- lets floating-
  // point rounding drift the norm away from 1 over enough repetitions,
  // even though each individual composition is exact to within a ULP.
  // Cheap enough (one rsqrt-scale operation) to call every tick rather
  // than track when it's actually needed.
  //
  // Substitutes Identity before normalizing wherever the rotation's own
  // squared norm is near zero, rather than calling Quaternion::
  // NormalizeInPlace directly on it: a genuinely zero quaternion shows up
  // on this engine's own SIMD-lane padding rows (kJointPosition's
  // "absent data reads as an inert default" convention zero-fills them,
  // not PaddingSeed's Identity), and
  // NormalizeInPlace's own divide-by-norm asserts on exactly that input.
  // A per-lane branchless select (rather than an if, since T may be a
  // batch mixing real and padding lanes together) keeps every real lane's
  // result identical to a plain NormalizeInPlace call.
  constexpr Transform<T>& NormalizeRotationInPlace() {
    auto near_zero = rotation_.SquaredNorm() < T{1e-12};
    rotation_ = Quaternion(
        util::Select(near_zero, T{1}, rotation_.W()),
        util::Select(near_zero, T{0}, rotation_.X()),
        util::Select(near_zero, T{0}, rotation_.Y()),
        util::Select(near_zero, T{0}, rotation_.Z())
    );
    rotation_.NormalizeInPlace();
    return *this;
  }

  constexpr std::tuple<Vector3, Quaternion> ToTuple() const {
    return std::make_tuple(translation_, rotation_);
  }

  friend constexpr Mask operator==(const Transform& a, const Transform& b) {
    return a.translation_ == b.translation_ && a.rotation_ == b.rotation_;
  }
  friend constexpr Mask operator!=(const Transform& a, const Transform& b) {
    return a.translation_ != b.translation_ || a.rotation_ != b.rotation_;
  }
  constexpr Mask IsApprox(const Transform& other, float epsilon = 1e-5) const {
    return translation_.IsApprox(other.translation_, epsilon) &&
           rotation_.IsApprox(other.rotation_, epsilon);
  }
  constexpr Mask IsZero(float epsilon = 1e-8) const {
    return translation_.IsZero(epsilon) &&
           rotation_.IsApprox(Quaternion::Identity(), epsilon);
  }

  constexpr Transform<T> operator*(const Transform<T>& other) const {
    return Transform<T>(
        rotation_.Rotate(other.translation_) + translation_,
        rotation_ * other.rotation_
    );
  }
  constexpr Transform<T>& operator*=(const Transform<T>& other) {
    translation_ = rotation_.Rotate(other.translation_) + translation_;
    rotation_ *= other.rotation_;
    return *this;
  }

  constexpr Transform<T> Inverse() const {
    Quaternion inv_rot = rotation_.Conjugate();
    return Transform<T>(inv_rot.Rotate(-translation_), inv_rot);
  }
  constexpr Transform<T>& InverseInPlace() {
    *this = Inverse();
    return *this;
  }

  // X m X^T, X = [R, t^ R; 0, R] the force transform: how a 6x6 operator
  // from motion to force (an inertia, a stiffness) moves to this
  // transform's destination frame. Worked block by block, with
  // m = [A, B; C, D]:
  //
  //   X m X^T = [P R^T + (Q R^T) t^T,  Q R^T;
  //              (R C) R^T + (R D R^T) t^T,  R D R^T],
  //   P = R A + t^ (R C),  Q = R B + t^ (R D),
  //
  // eight 3x3 products and four cross-product passes, rather than two 6x6
  // products.
  //
  // m: the operator, in this transform's source frame.
  // Returns: the operator in its destination frame.
  // Example: Matrix6x6 k_body = x_world.Inverse().Apply(k_world);
  constexpr math::Matrix6x6<T> Apply(const math::Matrix6x6<T>& m) const {
    const math::Matrix3x3<T> r = rotation_.ToRotationMatrix();
    const math::Matrix3x3<T> r_t = r.Transpose();
    const math::Matrix3x3<T> rc = r * Block(m, 3, 0);
    const math::Matrix3x3<T> rd = r * Block(m, 3, 3);
    const math::Matrix3x3<T> p = r * Block(m, 0, 0) + CrossColumns(rc);
    const math::Matrix3x3<T> q = r * Block(m, 0, 3) + CrossColumns(rd);
    const math::Matrix3x3<T> top_right = q * r_t;
    const math::Matrix3x3<T> bottom_right = rd * r_t;
    math::Matrix6x6<T> result;
    result.SetSubmatrix(0, 0, p * r_t + CrossRows(top_right));
    result.SetSubmatrix(0, 3, top_right);
    result.SetSubmatrix(3, 0, rc * r_t + CrossRows(bottom_right));
    result.SetSubmatrix(3, 3, bottom_right);
    return result;
  }
  constexpr SpatialVelocity<T> Apply(const SpatialVelocity<T>& v) const {
    Vector3 omega_dst = rotation_.Rotate(v.Angular());
    Vector3 v_dst =
        rotation_.Rotate(v.Linear()) + translation_.Cross(omega_dst);
    return {omega_dst, v_dst};
  }
  constexpr SpatialAcceleration<T> Apply(const SpatialAcceleration<T>& a
  ) const {
    Vector3 omega_dst = rotation_.Rotate(a.Angular());
    Vector3 v_dst =
        rotation_.Rotate(a.Linear()) + translation_.Cross(omega_dst);
    return {omega_dst, v_dst};
  }
  constexpr SpatialForce<T> Apply(const SpatialForce<T>& f) const {
    Vector3 f_dst = rotation_.Rotate(f.Linear());
    Vector3 tau_dst = rotation_.Rotate(f.Angular()) + translation_.Cross(f_dst);
    return {tau_dst, f_dst};
  }
  constexpr Inertia<T> Apply(const Inertia<T>& i) const {
    Vector3 h_dst = rotation_.Rotate(i.H()) + translation_ * i.Mass();
    math::Matrix3x3<T> r = rotation_.ToRotationMatrix();
    math::Matrix3x3<T> rotated_h_skew = r * i.H().Skew() * r.Transpose();
    math::Matrix3x3<T> translation_skew = translation_.Skew();
    math::Matrix3x3<T> i_dst = r * i.RotationalMatrix() * r.Transpose() +
                               h_dst.Skew() * translation_skew.Transpose() -
                               translation_skew * rotated_h_skew;
    return {i.Mass(), h_dst, i_dst};
  }
  constexpr InertiaOperator<T> Apply(const InertiaOperator<T>& i) const {
    return InertiaOperator<T>(Apply(i.AsMatrix()));
  }

 private:
  // The 3x3 block of m starting at (row, col).
  static constexpr math::Matrix3x3<T> Block(
      const math::Matrix6x6<T>& m, std::size_t row, std::size_t col
  ) {
    math::Matrix3x3<T> block;
    for (std::size_t i = 0; i < 3; ++i) {
      for (std::size_t j = 0; j < 3; ++j) {
        block(i, j) = m(row + i, col + j);
      }
    }
    return block;
  }

  // t^ y: this translation crossed with each column of y.
  constexpr math::Matrix3x3<T> CrossColumns(const math::Matrix3x3<T>& y) const {
    math::Matrix3x3<T> result;
    for (std::size_t j = 0; j < 3; ++j) {
      const Vector3 column =
          translation_.Cross(Vector3(y(0, j), y(1, j), y(2, j)));
      result(0, j) = column.X();
      result(1, j) = column.Y();
      result(2, j) = column.Z();
    }
    return result;
  }

  // y t^T: since t^T = -t^, row i is this translation crossed with row i
  // of y.
  constexpr math::Matrix3x3<T> CrossRows(const math::Matrix3x3<T>& y) const {
    math::Matrix3x3<T> result;
    for (std::size_t i = 0; i < 3; ++i) {
      const Vector3 row =
          translation_.Cross(Vector3(y(i, 0), y(i, 1), y(i, 2)));
      result(i, 0) = row.X();
      result(i, 1) = row.Y();
      result(i, 2) = row.Z();
    }
    return result;
  }

  Vector3 translation_;
  Quaternion rotation_;
};

template <typename T>
using TransformAssembler = engine::Assembler<
    Transform,
    T,
    math::Vector3Assembler<T>,
    math::QuaternionAssembler<T>>;
static_assert(engine::BatchAssemblerLike<TransformAssembler<float>>);

}  // namespace achilles::domain::spatial
