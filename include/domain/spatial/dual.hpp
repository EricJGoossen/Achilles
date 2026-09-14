#pragma once

#include <utility>

#include "domain/math/matrix.hpp"
#include "domain/math/vector3.hpp"
#include "domain/math/vector6.hpp"
#include "engine/assembler.hpp"
#include "util/simd_ops.hpp"

namespace achilles::domain::spatial {

template <template <typename> class DerivedT, util::ArithmeticLike T>
class Dual {
  using Mask = decltype(std::declval<T>() == std::declval<T>());
  using Derived = DerivedT<T>;

 public:
  using ScalarType = T;

  constexpr Dual() : data_() {}
  constexpr Dual(const math::Vector6<T>& data) : data_(data) {}
  constexpr Dual(T roll, T pitch, T yaw, T x, T y, T z)
      : data_(roll, pitch, yaw, x, y, z) {}
  constexpr Dual(
      const math::Vector3<T>& angular, const math::Vector3<T>& linear
  )
      : data_(
            angular.X(),
            angular.Y(),
            angular.Z(),
            linear.X(),
            linear.Y(),
            linear.Z()
        ) {}

  static constexpr Derived Zero() { return Derived{}; }
  constexpr Derived& SetZero() {
    data_.SetZero();
    return static_cast<Derived&>(*this);
  }

  static constexpr Derived UnitRoll() { return math::Vector6<T>::UnitA(); }
  static constexpr Derived UnitPitch() { return math::Vector6<T>::UnitB(); }
  static constexpr Derived UnitYaw() { return math::Vector6<T>::UnitC(); }
  static constexpr Derived UnitX() { return math::Vector6<T>::UnitD(); }
  static constexpr Derived UnitY() { return math::Vector6<T>::UnitE(); }
  static constexpr Derived UnitZ() { return math::Vector6<T>::UnitF(); }
  static constexpr Derived Ones() { return math::Vector6<T>::Ones(); }

  // Access
  inline constexpr std::tuple<math::Vector6<T>> ToTuple() const {
    return std::make_tuple(data_);
  }
  inline constexpr T Roll() const { return data_.A(); }
  inline constexpr T Pitch() const { return data_.B(); }
  inline constexpr T Yaw() const { return data_.C(); }
  inline constexpr T X() const { return data_.D(); }
  inline constexpr T Y() const { return data_.E(); }
  inline constexpr T Z() const { return data_.F(); }
  inline constexpr math::Vector3<T> Linear() const {
    return math::Vector3<T>(X(), Y(), Z());
  }
  inline constexpr math::Vector3<T> Angular() const {
    return math::Vector3<T>(Roll(), Pitch(), Yaw());
  }
  inline constexpr T operator[](std::size_t i) const { return data_[i]; }
  inline constexpr const math::Vector6<T>& AsVector6() const { return data_; }
  inline constexpr const math::Matrix<T, 6, 1>& AsMatrix() const {
    return data_.AsMatrix();
  }

  // Conversion
  template <template <typename> class OtherDerivedT>
  inline constexpr OtherDerivedT<T> As() const {
    return OtherDerivedT<T>(data_);
  }

  // Comparison
  friend inline constexpr Mask operator==(const Derived& a, const Derived& b) {
    return a.data_ == b.data_;
  }
  friend inline constexpr Mask operator!=(const Derived& a, const Derived& b) {
    return a.data_ != b.data_;
  }
  inline constexpr Mask IsApprox(const Derived& other, float epsilon = 1e-5)
      const {
    return data_.IsApprox(other.data_, epsilon);
  }
  inline constexpr Mask IsZero(float epsilon = 1e-8) const {
    return data_.IsZero(epsilon);
  }

  // Elementwise Arithmetic
  inline constexpr Derived operator+(const Derived& other) const {
    return data_ + other.data_;
  }
  inline constexpr Derived operator-(const Derived& other) const {
    return data_ - other.data_;
  }
  inline constexpr Derived operator-() const { return -data_; }
  inline constexpr Derived& NegateInPlace() {
    data_.NegateInPlace();
    return static_cast<Derived&>(*this);
  }
  inline constexpr Derived& operator+=(const Derived& other) {
    data_ += other.data_;
    return static_cast<Derived&>(*this);
  }
  inline constexpr Derived& operator-=(const Derived& other) {
    data_ -= other.data_;
    return static_cast<Derived&>(*this);
  }

  // Scalar Algebra
  inline constexpr Derived operator*(T scalar) const { return data_ * scalar; }
  inline constexpr Derived operator/(T scalar) const { return data_ / scalar; }
  inline constexpr Derived& operator*=(T scalar) {
    data_ *= scalar;
    return static_cast<Derived&>(*this);
  }
  inline constexpr Derived& operator/=(T scalar) {
    data_ /= scalar;
    return static_cast<Derived&>(*this);
  }

  // Products
  inline constexpr T Dot(const Derived& other) const {
    return data_.Dot(other.data_);
  }

  // Norms
  inline constexpr T Norm() const { return data_.Norm(); }
  inline constexpr T SquaredNorm() const { return data_.SquaredNorm(); }
  inline constexpr Derived Normalize() const { return data_.Normalize(); }
  inline constexpr Derived& NormalizeInPlace() {
    data_.NormalizeInPlace();
    return static_cast<Derived&>(*this);
  }

  // Geometry
  inline constexpr Derived ProjectOnto(const Derived& other) const {
    T sq = other.SquaredNorm();
    return sq > T{0} ? other * (Dot(other) / sq) : Derived::Zero();
  }

  // Interpolation
  inline static constexpr Derived Lerp(
      const Derived& a, const Derived& b, T t
  ) {
    return a + (b - a) * t;
  }

  // Friend functions
  friend inline constexpr Derived operator*(T scalar, const Derived& v) {
    return {v * scalar};
  }
  friend inline constexpr Derived operator*(
      const math::Matrix6x6<T>& m, const Derived& v
  ) {
    return {m * v.AsMatrix()};
  }

 private:
  math::Vector6<T> data_;
};

// Forward declarations of spatial types
template <util::ArithmeticLike T>
class SpatialVector;
template <util::ArithmeticLike T>
class SpatialVelocity;
template <util::ArithmeticLike T>
class SpatialAcceleration;
template <util::ArithmeticLike T>
class SpatialMomentum;
template <util::ArithmeticLike T>
class SpatialForce;

// Type aliases for spatial types
template <util::ArithmeticLike T>
class SpatialPosition : public Dual<SpatialPosition, T> {
 public:
  using Dual<SpatialPosition, T>::Dual;
};
template <typename T>
using SpatialPositionAssembler =
    engine::Assembler<SpatialPosition, T, math::Vector6Assembler<T>>;
static_assert(engine::AssemblerLike<SpatialPositionAssembler<float>>);

template <util::ArithmeticLike T>
class SpatialVelocity : public Dual<SpatialVelocity, T> {
 public:
  using Dual<SpatialVelocity, T>::Dual;

  constexpr SpatialForce<T> CrossForce(const SpatialMomentum<T>& f) const {
    return {
        this->Angular().Cross(f.Angular()) + this->Linear().Cross(f.Linear()),
        this->Angular().Cross(f.Linear())
    };
  }

  inline constexpr SpatialAcceleration<T> Cross(const SpatialVelocity& other
  ) const {
    return {
        this->Angular().Cross(other.Angular()),
        this->Angular().Cross(other.Linear()) +
            this->Linear().Cross(other.Angular())
    };
  }
};
template <typename T>
using SpatialVelocityAssembler =
    engine::Assembler<SpatialVelocity, T, math::Vector6Assembler<T>>;
static_assert(engine::AssemblerLike<SpatialVelocityAssembler<float>>);

template <util::ArithmeticLike T>
class SpatialAcceleration : public Dual<SpatialAcceleration, T> {
 public:
  using Dual<SpatialAcceleration, T>::Dual;
};
template <typename T>
using SpatialAccelerationAssembler =
    engine::Assembler<SpatialAcceleration, T, math::Vector6Assembler<T>>;
static_assert(engine::AssemblerLike<SpatialAccelerationAssembler<float>>);

template <util::ArithmeticLike T>
class SpatialMomentum : public Dual<SpatialMomentum, T> {
 public:
  using Dual<SpatialMomentum, T>::Dual;
};
template <typename T>
using SpatialMomentumAssembler =
    engine::Assembler<SpatialMomentum, T, math::Vector6Assembler<T>>;
static_assert(engine::AssemblerLike<SpatialMomentumAssembler<float>>);

template <util::ArithmeticLike T>
class SpatialForce : public Dual<SpatialForce, T> {
 public:
  using Dual<SpatialForce, T>::Dual;

  inline constexpr T Dot(const spatial::SpatialAcceleration<T>& a) const {
    return this->AsVector6().Dot(a.AsVector6());
  }
};
template <typename T>
using SpatialForceAssembler =
    engine::Assembler<SpatialForce, T, math::Vector6Assembler<T>>;
static_assert(engine::AssemblerLike<SpatialForceAssembler<float>>);

}  // namespace achilles::domain::spatial
