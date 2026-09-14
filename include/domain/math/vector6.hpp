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
class Vector6 {
  using Mask = decltype(std::declval<T>() == std::declval<T>());

 public:
  using ScalarType = T;

  constexpr Vector6() : data_() {}
  constexpr Vector6(const Matrix<T, 6, 1>& data) : data_(data) {}
  constexpr Vector6(T a, T b, T c, T d, T e, T f) : data_(a, b, c, d, e, f) {}

  static constexpr Vector6 Zero() { return Vector6{}; }
  constexpr Vector6& SetZero() {
    data_.SetZero();
    return *this;
  }

  static constexpr Vector6 UnitA() {
    return Vector6(T{1}, T{0}, T{0}, T{0}, T{0}, T{0});
  }
  static constexpr Vector6 UnitB() {
    return Vector6(T{0}, T{1}, T{0}, T{0}, T{0}, T{0});
  }
  static constexpr Vector6 UnitC() {
    return Vector6(T{0}, T{0}, T{1}, T{0}, T{0}, T{0});
  }
  static constexpr Vector6 UnitD() {
    return Vector6(T{0}, T{0}, T{0}, T{1}, T{0}, T{0});
  }
  static constexpr Vector6 UnitE() {
    return Vector6(T{0}, T{0}, T{0}, T{0}, T{1}, T{0});
  }
  static constexpr Vector6 UnitF() {
    return Vector6(T{0}, T{0}, T{0}, T{0}, T{0}, T{1});
  }
  static constexpr Vector6 Ones() { return Matrix<T, 6, 1>::Ones(); }

  // Access
  inline constexpr std::tuple<T, T, T, T, T, T> ToTuple() const {
    return std::make_tuple(A(), B(), C(), D(), E(), F());
  }
  inline constexpr T A() const { return data_[0]; }
  inline constexpr T B() const { return data_[1]; }
  inline constexpr T C() const { return data_[2]; }
  inline constexpr T D() const { return data_[3]; }
  inline constexpr T E() const { return data_[4]; }
  inline constexpr T F() const { return data_[5]; }
  inline constexpr T operator[](std::size_t i) const { return data_[i]; }
  inline constexpr const Matrix<T, 6, 1>& AsMatrix() const { return data_; }

  // Comparison
  friend inline constexpr Mask operator==(const Vector6& a, const Vector6& b) {
    return a.data_ == b.data_;
  }
  friend inline constexpr Mask operator!=(const Vector6& a, const Vector6& b) {
    return a.data_ != b.data_;
  }
  inline constexpr Mask IsApprox(const Vector6& other, float epsilon = 1e-5)
      const {
    return data_.IsApprox(other.data_, epsilon);
  }
  inline constexpr Mask IsZero(float epsilon = 1e-8) const {
    return data_.IsZero(epsilon);
  }

  // Elementwise Arithmetic
  inline constexpr Vector6 operator+(const Vector6<T>& other) const {
    return data_ + other.data_;
  }
  inline constexpr Vector6 operator-(const Vector6<T>& other) const {
    return data_ - other.data_;
  }
  inline constexpr Vector6 operator-() const { return -data_; }
  inline constexpr Vector6& NegateInPlace() {
    data_.NegateInPlace();
    return *this;
  }
  inline constexpr Vector6& operator+=(const Vector6& other) {
    data_ += other.data_;
    return *this;
  }
  inline constexpr Vector6& operator-=(const Vector6& other) {
    data_ -= other.data_;
    return *this;
  }

  // Scalar Algebra
  inline constexpr Vector6 operator*(T scalar) const { return data_ * scalar; }
  inline constexpr Vector6 operator/(T scalar) const { return data_ / scalar; }
  inline constexpr Vector6& operator*=(T scalar) {
    data_ *= scalar;
    return *this;
  }
  inline constexpr Vector6& operator/=(T scalar) {
    data_ /= scalar;
    return *this;
  }

  // Products
  inline constexpr T Dot(const Vector6& other) const {
    return data_.Dot(other.data_);
  }

  // Norms
  inline constexpr T Norm() const { return data_.Norm(); }
  inline constexpr T SquaredNorm() const { return data_.NormSquared(); }
  inline constexpr Vector6 Normalize() const { return data_.Normalize(); }
  inline constexpr Vector6& NormalizeInPlace() {
    data_.NormalizeInPlace();
    return *this;
  }

  // Geometry
  inline constexpr Vector6 ProjectOnto(const Vector6& other) const {
    T sq = other.SquaredNorm();
    return sq > T{0} ? other * (Dot(other) / sq) : Vector6::Zero();
  }

  // Interpolation
  inline static constexpr Vector6 Lerp(
      const Vector6& a, const Vector6& b, T t
  ) {
    return a + (b - a) * t;
  }

  // Friend functions
  friend inline constexpr Vector6 operator*(T scalar, const Vector6& v) {
    return v * scalar;
  }
  friend inline constexpr Vector6 operator*(
      const Matrix<T, 6, 6>& m, const Vector6& v
  ) {
    return m * v.AsMatrix();
  }

 private:
  Matrix<T, 6, 1> data_;
};

template <typename T>
using Vector6Assembler = engine::Assembler<Vector6, T, engine::RepeatTypes<6>>;
static_assert(engine::AssemblerLike<Vector6Assembler<float>>);

}  // namespace achilles::domain::math
