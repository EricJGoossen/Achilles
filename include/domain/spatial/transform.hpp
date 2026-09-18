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

  constexpr math::Matrix6x6<T> Apply(const math::Matrix6x6<T>& m) const {
    math::Matrix3x3<T> r = rotation_.ToRotationMatrix();
    math::Matrix3x3<T> t = translation_.Skew();
    math::Matrix6x6<T> x;
    x.SetSubmatrix(0, 0, r);
    x.SetSubmatrix(0, 3, t * r);
    x.SetSubmatrix(3, 0, math::Matrix3x3<T>::Zero());
    x.SetSubmatrix(3, 3, r);
    return x * m * x.Transpose();
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
