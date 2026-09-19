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
  constexpr explicit Dual(const math::Vector6<T>& data) : data_(data) {}
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

  static constexpr Derived UnitRoll() {
    return Derived(math::Vector6<T>::UnitA());
  }
  static constexpr Derived UnitPitch() {
    return Derived(math::Vector6<T>::UnitB());
  }
  static constexpr Derived UnitYaw() {
    return Derived(math::Vector6<T>::UnitC());
  }
  static constexpr Derived UnitX() {
    return Derived(math::Vector6<T>::UnitD());
  }
  static constexpr Derived UnitY() {
    return Derived(math::Vector6<T>::UnitE());
  }
  static constexpr Derived UnitZ() {
    return Derived(math::Vector6<T>::UnitF());
  }
  static constexpr Derived Ones() { return Derived(math::Vector6<T>::Ones()); }

  static constexpr Derived PaddingSeed() { return Zero(); }

  // Access
  constexpr std::tuple<math::Vector6<T>> ToTuple() const {
    return std::make_tuple(data_);
  }
  constexpr T Roll() const { return data_.A(); }
  constexpr T Pitch() const { return data_.B(); }
  constexpr T Yaw() const { return data_.C(); }
  constexpr T X() const { return data_.D(); }
  constexpr T Y() const { return data_.E(); }
  constexpr T Z() const { return data_.F(); }
  constexpr math::Vector3<T> Linear() const {
    return math::Vector3<T>(X(), Y(), Z());
  }
  constexpr math::Vector3<T> Angular() const {
    return math::Vector3<T>(Roll(), Pitch(), Yaw());
  }
  constexpr T operator[](std::size_t i) const { return data_[i]; }
  constexpr const math::Vector6<T>& AsVector6() const { return data_; }
  constexpr const math::Matrix<T, 6, 1>& AsMatrix() const {
    return data_.AsMatrix();
  }

  // Conversion
  template <template <typename> class OtherDerivedT>
  constexpr OtherDerivedT<T> As() const {
    return OtherDerivedT<T>(data_);
  }

  // Comparison
  friend constexpr Mask operator==(const Derived& a, const Derived& b) {
    return a.data_ == b.data_;
  }
  friend constexpr Mask operator!=(const Derived& a, const Derived& b) {
    return a.data_ != b.data_;
  }
  constexpr Mask IsApprox(const Derived& other, float epsilon = 1e-5) const {
    return data_.IsApprox(other.data_, epsilon);
  }
  constexpr Mask IsZero(float epsilon = 1e-8) const {
    return data_.IsZero(epsilon);
  }

  // Elementwise Arithmetic
  constexpr Derived operator+(const Derived& other) const {
    return Derived(data_ + other.data_);
  }
  constexpr Derived operator-(const Derived& other) const {
    return Derived(data_ - other.data_);
  }
  constexpr Derived operator-() const { return Derived(-data_); }
  constexpr Derived& NegateInPlace() {
    data_.NegateInPlace();
    return static_cast<Derived&>(*this);
  }
  constexpr Derived& operator+=(const Derived& other) {
    data_ += other.data_;
    return static_cast<Derived&>(*this);
  }
  constexpr Derived& operator-=(const Derived& other) {
    data_ -= other.data_;
    return static_cast<Derived&>(*this);
  }

  // Scalar Algebra
  constexpr Derived operator*(T scalar) const {
    return Derived(data_ * scalar);
  }
  friend constexpr Derived operator*(T scalar, const Derived& v) {
    return v * scalar;
  }
  constexpr Derived operator/(T scalar) const {
    return Derived(data_ / scalar);
  }
  constexpr Derived& operator*=(T scalar) {
    data_ *= scalar;
    return static_cast<Derived&>(*this);
  }
  constexpr Derived& operator/=(T scalar) {
    data_ /= scalar;
    return static_cast<Derived&>(*this);
  }

  // Products
  constexpr T Dot(const Derived& other) const { return data_.Dot(other.data_); }
  friend constexpr Derived operator*(
      const math::Matrix6x6<T>& m, const Derived& v
  ) {
    return Derived(math::Vector6<T>(m * v.AsMatrix()));
  }

  // Norms
  constexpr T Norm() const { return data_.Norm(); }
  constexpr T SquaredNorm() const { return data_.SquaredNorm(); }
  constexpr Derived Normalize() const { return Derived(data_.Normalize()); }
  constexpr Derived& NormalizeInPlace() {
    data_.NormalizeInPlace();
    return static_cast<Derived&>(*this);
  }

  // Geometry
  constexpr Derived ProjectOnto(const Derived& other) const {
    T sq = other.SquaredNorm();
    return util::Select(sq > T{0}, other * (Dot(other) / sq), Derived::Zero());
  }

  // Interpolation
  static constexpr Derived Lerp(const Derived& a, const Derived& b, T t) {
    return a + (b - a) * t;
  }

  // Printing
  friend std::ostream& operator<<(
      std::ostream& os, const spatial::Dual<DerivedT, T>& d
  ) {
    os << "Dual(" << d.Linear() << ", " << d.Angular() << ")";
    return os;
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

  constexpr SpatialAcceleration<T> Cross(const SpatialVelocity& other) const {
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

  constexpr SpatialVelocity<T> Integrate(T dt) const {
    return SpatialVelocity<T>(this->AsVector6() * dt);
  }
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
  using Dual<SpatialForce, T>::Dot;

  constexpr T Dot(const spatial::SpatialAcceleration<T>& a) const {
    return this->AsVector6().Dot(a.AsVector6());
  }
};
template <typename T>
using SpatialForceAssembler =
    engine::Assembler<SpatialForce, T, math::Vector6Assembler<T>>;
static_assert(engine::AssemblerLike<SpatialForceAssembler<float>>);

}  // namespace achilles::domain::spatial
