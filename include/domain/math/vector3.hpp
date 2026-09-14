#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <xsimd/xsimd.hpp>

#include "engine/assembler.hpp"
#include "matrix.hpp"
#include "util/simd_ops.hpp"

namespace achilles::domain::math {

template <util::ArithmeticLike T>
class Vector3 {
  using Mask = decltype(std::declval<T>() == std::declval<T>());

 public:
  using ScalarType = T;

  constexpr Vector3() : data_() {}
  constexpr Vector3(const Matrix<T, 3, 1>& data) : data_(data) {}
  constexpr Vector3(T x, T y, T z) : data_(x, y, z) {}

  static constexpr Vector3 Zero() { return Vector3{}; }
  constexpr Vector3& SetZero() {
    data_.SetZero();
    return *this;
  }
  static constexpr Vector3 FromSkew(const Matrix<T, 3, 3>& skew) {
    return Vector3(skew(2, 1), skew(0, 2), skew(1, 0));
  }

  static constexpr Vector3 UnitX() { return Vector3(T{1}, T{0}, T{0}); }
  static constexpr Vector3 UnitY() { return Vector3(T{0}, T{1}, T{0}); }
  static constexpr Vector3 UnitZ() { return Vector3(T{0}, T{0}, T{1}); }
  static constexpr Vector3 Ones() { return Matrix<T, 3, 1>::Ones(); }

  // Access
  inline constexpr std::tuple<T, T, T> ToTuple() const {
    return std::make_tuple(X(), Y(), Z());
  }
  inline constexpr T X() const { return data_[0]; }
  inline constexpr T Y() const { return data_[1]; }
  inline constexpr T Z() const { return data_[2]; }
  inline constexpr T operator[](std::size_t i) const { return data_[i]; }
  inline constexpr const Matrix<T, 3, 1>& AsMatrix() const { return data_; }

  // Comparison
  friend inline constexpr Mask operator==(const Vector3& a, const Vector3& b) {
    return a.data_ == b.data_;
  }
  friend inline constexpr Mask operator!=(const Vector3& a, const Vector3& b) {
    return a.data_ != b.data_;
  }
  inline constexpr Mask IsApprox(const Vector3& other, float epsilon = 1e-5)
      const {
    return data_.IsApprox(other.data_, epsilon);
  }
  inline constexpr Mask IsZero(float epsilon = 1e-8) const {
    return data_.IsZero(epsilon);
  }

  // Elementwise Arithmetic
  inline constexpr Vector3 operator+(const Vector3<T>& other) const {
    return data_ + other.data_;
  }
  inline constexpr Vector3 operator-(const Vector3<T>& other) const {
    return data_ - other.data_;
  }
  inline constexpr Vector3 operator-() const { return -data_; }
  inline constexpr Vector3& NegateInPlace() {
    data_.NegateInPlace();
    return *this;
  }
  inline constexpr Vector3& operator+=(const Vector3& other) {
    data_ += other.data_;
    return *this;
  }
  inline constexpr Vector3& operator-=(const Vector3& other) {
    data_ -= other.data_;
    return *this;
  }

  // Scalar Algebra
  inline constexpr Vector3 operator*(T scalar) const { return data_ * scalar; }
  inline constexpr Vector3 operator/(T scalar) const { return data_ / scalar; }
  inline constexpr Vector3& operator*=(T scalar) {
    data_ *= scalar;
    return *this;
  }
  inline constexpr Vector3& operator/=(T scalar) {
    data_ /= scalar;
    return *this;
  }

  // Products
  inline constexpr T Dot(const Vector3& other) const {
    return data_.Dot(other.data_);
  }
  inline constexpr Vector3 Cross(const Vector3& other) const {
    return Vector3(
        Y() * other.Z() - Z() * other.Y(),
        Z() * other.X() - X() * other.Z(),
        X() * other.Y() - Y() * other.X()
    );
  }

  // Norms
  inline constexpr T Norm() const { return data_.Norm(); }
  inline constexpr T SquaredNorm() const { return data_.NormSquared(); }
  inline constexpr Vector3 Normalize() const { return data_.Normalize(); }
  inline constexpr Vector3& NormalizeInPlace() {
    data_.NormalizeInPlace();
    return *this;
  }

  // Geometry
  inline constexpr T AngleTo(const Vector3& other) const {
    T denom = Norm() * other.Norm();
    return xsimd::select(
        denom > T{0},
        std::acos(std::clamp(Dot(other) / denom, T{-1}, T{1})),
        T{0}
    );
  }
  inline constexpr Vector3 ProjectOnto(const Vector3& other) const {
    T sq = other.SquaredNorm();
    return xsimd::select(sq > T{0}, other * (Dot(other) / sq), Vector3::Zero());
  }
  inline constexpr Matrix<T, 3, 3> Skew() const {
    return Matrix<T, 3, 3>(T{0}, -Z(), Y(), Z(), T{0}, -X(), -Y(), X(), T{0});
  }

  // Interpolation
  inline static constexpr Vector3 Lerp(
      const Vector3& a, const Vector3& b, T t
  ) {
    return a + (b - a) * t;
  }

  // Friend functions
  friend inline constexpr Vector3 operator*(T scalar, const Vector3& v) {
    return v * scalar;
  }
  friend inline constexpr Vector3 operator*(
      const Matrix<T, 3, 3>& m, const Vector3& v
  ) {
    return m * v.AsMatrix();
  }

 private:
  Matrix<T, 3, 1> data_;
};

template <typename T>
using Vector3Assembler = engine::Assembler<Vector3, T, engine::RepeatTypes<3>>;
static_assert(engine::AssemblerLike<Vector3Assembler<float>>);

}  // namespace achilles::domain::math
