#pragma once

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

  // X m X^T, with X = [[R, t~R], [0, R]] the 6x6 force transform (t~ the
  // skew matrix of the translation). Computed block-wise on m's own 3x3
  // blocks [[A, B], [C, D]] instead of as two dense 6x6 products: X's
  // zero block and the t~ factors (a t~ product is just a cross product
  // per row/column) are never multiplied out, which cuts the work by
  // about a third -- this runs once per joint in ABA's inertia pass.
  //   X m     = [[R A + t~ R C, R B + t~ R D], [R C, R D]]
  //           = [[P11, P12], [P21, P22]]
  //   X m X^T = [[P11 R^T + P12 R^T t~^T, P12 R^T],
  //              [P21 R^T + P22 R^T t~^T, P22 R^T]]
  constexpr math::Matrix6x6<T> Apply(const math::Matrix6x6<T>& m) const {
    using Matrix3x3 = math::Matrix3x3<T>;
    const Matrix3x3 r = rotation_.ToRotationMatrix();
    const Matrix3x3 r_t = r.Transpose();

    const Matrix3x3 p21 = r * m.template Submatrix<3, 3>(3, 0);
    const Matrix3x3 p22 = r * m.template Submatrix<3, 3>(3, 3);
    const Matrix3x3 p11 = r * m.template Submatrix<3, 3>(0, 0) + SkewTimes(p21);
    const Matrix3x3 p12 = r * m.template Submatrix<3, 3>(0, 3) + SkewTimes(p22);

    const Matrix3x3 q12 = p12 * r_t;
    const Matrix3x3 q22 = p22 * r_t;

    math::Matrix6x6<T> x;
    x.SetSubmatrix(0, 0, p11 * r_t + TimesSkewTranspose(q12));
    x.SetSubmatrix(0, 3, q12);
    x.SetSubmatrix(3, 0, p21 * r_t + TimesSkewTranspose(q22));
    x.SetSubmatrix(3, 3, q22);
    return x;
  }

  // Apply(m) for a symmetric m -- an articulated/rigid-body inertia -- at
  // about two thirds the cost: only m's upper triangle is read (its lower-
  // left block is taken as the upper-right's transpose), and only the
  // result's upper triangle is computed, then mirrored. With
  // m = [[A, B], [B^T, D]], A' = R A R^T, B' = R B R^T, D' = R D R^T:
  //   X m X^T = [[A' + t~ B'^T + F t~^T, F], [F^T, D']],  F = B' + t~ D'
  // (exactly what Apply(m) gives for that m, up to float rounding, and
  // exactly symmetric). always_inline: it sits on ABA's hottest path, and
  // out of line its 6x6 argument and result each cross the call through
  // memory.
  [[gnu::always_inline]] constexpr math::Matrix6x6<T> ApplySymmetric(
      const math::Matrix6x6<T>& m
  ) const {
    using Matrix3x3 = math::Matrix3x3<T>;
    const Matrix3x3 r = rotation_.ToRotationMatrix();
    const Matrix3x3 a = RotateSymmetric(r, m, 0);
    const Matrix3x3 d = RotateSymmetric(r, m, 3);
    const Matrix3x3 b = r * m.template Submatrix<3, 3>(0, 3) * r.Transpose();

    const Matrix3x3 f = b + SkewTimes(d);
    const Matrix3x3 b_t_skew = TimesSkewTranspose(b);
    const Matrix3x3 f_t_skew = TimesSkewTranspose(f);

    math::Matrix6x6<T> x;
    for (std::size_t i = 0; i < 3; ++i) {
      for (std::size_t j = i; j < 3; ++j) {
        // (t~ B'^T)(i, j) = (B' t~^T)(j, i)
        const T top_left = a(i, j) + b_t_skew(j, i) + f_t_skew(i, j);
        x(i, j) = top_left;
        x(j, i) = top_left;
        x(i + 3, j + 3) = d(i, j);
        x(j + 3, i + 3) = d(i, j);
      }
      for (std::size_t j = 0; j < 3; ++j) {
        x(i, j + 3) = f(i, j);
        x(j + 3, i) = f(i, j);
      }
    }
    return x;
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
  // R S R^T for the symmetric 3x3 diagonal block of `m` starting at
  // (offset, offset); only that block's upper triangle is read, and only
  // the result's is computed, then mirrored.
  static constexpr math::Matrix3x3<T> RotateSymmetric(
      const math::Matrix3x3<T>& r,
      const math::Matrix6x6<T>& m,
      std::size_t offset
  ) {
    auto s = [&](std::size_t i, std::size_t j) {
      return i <= j ? m(offset + i, offset + j) : m(offset + j, offset + i);
    };
    // rs = R S
    math::Matrix3x3<T> rs;
    for (std::size_t i = 0; i < 3; ++i) {
      for (std::size_t j = 0; j < 3; ++j) {
        rs(i, j) = r(i, 0) * s(0, j) + r(i, 1) * s(1, j) + r(i, 2) * s(2, j);
      }
    }
    math::Matrix3x3<T> result;
    for (std::size_t i = 0; i < 3; ++i) {
      for (std::size_t j = i; j < 3; ++j) {
        const T value =
            rs(i, 0) * r(j, 0) + rs(i, 1) * r(j, 1) + rs(i, 2) * r(j, 2);
        result(i, j) = value;
        result(j, i) = value;
      }
    }
    return result;
  }

  // t~ m: the translation crossed into each column of m.
  constexpr math::Matrix3x3<T> SkewTimes(const math::Matrix3x3<T>& m) const {
    const T tx = translation_.X();
    const T ty = translation_.Y();
    const T tz = translation_.Z();
    math::Matrix3x3<T> result;
    for (std::size_t j = 0; j < 3; ++j) {
      result(0, j) = ty * m(2, j) - tz * m(1, j);
      result(1, j) = tz * m(0, j) - tx * m(2, j);
      result(2, j) = tx * m(1, j) - ty * m(0, j);
    }
    return result;
  }

  // m t~^T: the translation crossed into each row of m.
  constexpr math::Matrix3x3<T> TimesSkewTranspose(const math::Matrix3x3<T>& m
  ) const {
    const T tx = translation_.X();
    const T ty = translation_.Y();
    const T tz = translation_.Z();
    math::Matrix3x3<T> result;
    for (std::size_t i = 0; i < 3; ++i) {
      result(i, 0) = ty * m(i, 2) - tz * m(i, 1);
      result(i, 1) = tz * m(i, 0) - tx * m(i, 2);
      result(i, 2) = tx * m(i, 1) - ty * m(i, 0);
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
static_assert(engine::AssemblerLike<TransformAssembler<float>>);

}  // namespace achilles::domain::spatial
