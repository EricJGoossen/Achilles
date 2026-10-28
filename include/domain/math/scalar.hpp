#pragma once

#include <cstdint>
#include <ostream>
#include <tuple>
#include <xsimd/xsimd.hpp>

#include "engine/assembler.hpp"
#include "util/simd_ops.hpp"

namespace achilles::domain::math {

// One value per instance, as a field type in its own right -- what a
// per-body quantity with no further structure (a margin, a radius, a row
// index, ...) is stored and handed to an Op as, rather than borrowing a
// 1x1 Matrix or a spare component of some wider type. T is a plain scalar
// or an xsimd batch of one, the same as every other domain type here;
// integer T works too (e.g. a row index stored as int32 lanes).
template <util::ArithmeticLike T>
class Scalar {
  using Mask = decltype(std::declval<T>() == std::declval<T>());

 public:
  using ScalarType = T;

  constexpr Scalar() : value_() {}
  constexpr explicit Scalar(T value) : value_(value) {}

  static constexpr Scalar Zero() { return Scalar{}; }
  constexpr Scalar& SetZero() {
    value_ = T{0};
    return *this;
  }
  static constexpr Scalar PaddingSeed() { return Zero(); }

  // Access
  constexpr std::tuple<T> ToTuple() const { return std::make_tuple(value_); }
  constexpr T Value() const { return value_; }

  // Comparison
  friend constexpr Mask operator==(const Scalar& a, const Scalar& b) {
    return a.value_ == b.value_;
  }
  friend constexpr Mask operator!=(const Scalar& a, const Scalar& b) {
    return a.value_ != b.value_;
  }
  friend constexpr Mask operator<(const Scalar& a, const Scalar& b) {
    return a.value_ < b.value_;
  }
  friend constexpr Mask operator<=(const Scalar& a, const Scalar& b) {
    return a.value_ <= b.value_;
  }
  friend constexpr Mask operator>(const Scalar& a, const Scalar& b) {
    return a.value_ > b.value_;
  }
  friend constexpr Mask operator>=(const Scalar& a, const Scalar& b) {
    return a.value_ >= b.value_;
  }

  // Arithmetic
  constexpr Scalar operator+(const Scalar& other) const {
    return Scalar(value_ + other.value_);
  }
  constexpr Scalar operator-(const Scalar& other) const {
    return Scalar(value_ - other.value_);
  }
  constexpr Scalar operator*(const Scalar& other) const {
    return Scalar(value_ * other.value_);
  }
  constexpr Scalar operator/(const Scalar& other) const {
    return Scalar(value_ / other.value_);
  }
  constexpr Scalar operator-() const { return Scalar(-value_); }
  constexpr Scalar& operator+=(const Scalar& other) {
    value_ += other.value_;
    return *this;
  }
  constexpr Scalar& operator-=(const Scalar& other) {
    value_ -= other.value_;
    return *this;
  }
  constexpr Scalar& operator*=(const Scalar& other) {
    value_ *= other.value_;
    return *this;
  }
  constexpr Scalar& operator/=(const Scalar& other) {
    value_ /= other.value_;
    return *this;
  }

  // Printing
  friend std::ostream& operator<<(std::ostream& os, const Scalar& s) {
    os << "Scalar(" << s.value_ << ")";
    return os;
  }

 private:
  T value_;
};

template <typename T>
using ScalarAssembler = engine::Assembler<Scalar, T, engine::RepeatTypes<1>>;
static_assert(engine::BatchAssemblerLike<ScalarAssembler<float>>);
static_assert(engine::BatchAssemblerLike<ScalarAssembler<std::int32_t>>);

}  // namespace achilles::domain::math
