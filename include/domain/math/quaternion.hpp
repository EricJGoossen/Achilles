#pragma once

#include <cmath>
#include <xsimd/xsimd.hpp>

#include "engine/assembler.hpp"
#include "matrix.hpp"
#include "util/simd_ops.hpp"
#include "vector3.hpp"

namespace achilles::domain::math {

template <util::ArithmeticLike T>
class Quaternion {
  using Mask = decltype(std::declval<T>() == std::declval<T>());

 public:
  using ScalarType = T;

  constexpr Quaternion() : data_(T{1}, T{0}, T{0}, T{0}) {}
  constexpr Quaternion(T w, T x, T y, T z) : data_(w, x, y, z) {}
  constexpr explicit Quaternion(const Matrix<T, 4, 1>& data) : data_(data) {}

  static constexpr Quaternion<T> FromVector(const Vector3<T>& v) {
    return Quaternion<T>(T{0}, v.X(), v.Y(), v.Z()).NormalizeInPlace();
  }
  static constexpr Quaternion<T> Identity() { return Quaternion<T>(); }
  constexpr Quaternion<T>& SetIdentity() {
    *this = Identity();
    return *this;
  }
  static constexpr Quaternion<T> Exp(const Vector3<T>& v) {
    using std::cos;
    using std::sin;
    using xsimd::cos;
    using xsimd::sin;

    T kEpsilon = T{1e-8};
    T theta = v.Norm();

    // theta == 0 (no rotation) is a removable singularity in
    // k = sin(theta/2)/theta -- its limit as theta -> 0 is 1/2, so
    // Exp(0) is exactly Identity(), not undefined. Substitute a safe
    // nonzero divisor for just the division (its result is discarded
    // below wherever the mask says near-zero) rather than branching, so
    // this stays correct per-lane for a batched T -- same technique
    // SymmetricEigenvalues (domain/spatial/inertia.hpp) uses for its own
    // analogous 0/0 case.
    auto near_zero = theta <= kEpsilon;
    T safe_theta = util::Select(near_zero, T{1}, theta);

    T half_theta = theta * T{0.5};
    T sin_half_theta = sin(half_theta);
    T k = sin_half_theta / safe_theta;

    return Quaternion<T>(
        util::Select(near_zero, T{1}, cos(half_theta)),
        util::Select(near_zero, T{0}, k * v.X()),
        util::Select(near_zero, T{0}, k * v.Y()),
        util::Select(near_zero, T{0}, k * v.Z())
    );
  }
  static constexpr Quaternion<T> PaddingSeed() { return Identity(); }

  // Access
  constexpr std::tuple<T, T, T, T> ToTuple() const {
    return std::make_tuple(W(), X(), Y(), Z());
  }
  constexpr T W() const { return data_[0]; }
  constexpr T X() const { return data_[1]; }
  constexpr T Y() const { return data_[2]; }
  constexpr T Z() const { return data_[3]; }
  constexpr T operator[](std::size_t i) const { return data_[i]; }
  constexpr const Matrix<T, 4, 1>& AsMatrix() const { return data_; }
  constexpr Matrix<T, 3, 3> ToRotationMatrix() const {
    T w = W();
    T x = X();
    T y = Y();
    T z = Z();
    // clang-format off
    return Matrix<T, 3, 3>{
      T{1} - T{2} * (y * y + z * z),        T{2} * (x * y - w * z),        T{2} * (x * z + w * y),
             T{2} * (x * y + w * z), T{1} - T{2} * (x * x + z * z),        T{2} * (y * z - w * x),
             T{2} * (x * z - w * y),        T{2} * (y * z + w * x), T{1} - T{2} * (x * x + y * y)
    };
    // clang-format on
  }

  // Comparison
  friend constexpr Mask operator==(const Quaternion& a, const Quaternion& b) {
    return a.data_ == b.data_;
  }
  friend constexpr Mask operator!=(const Quaternion& a, const Quaternion& b) {
    return a.data_ != b.data_;
  }
  constexpr Mask IsApprox(const Quaternion& other, float epsilon = 1e-5) const {
    return data_.IsApprox(other.data_, epsilon);
  }
  constexpr Mask IsIdentity(float epsilon = 1e-8) const {
    return this->IsApprox(Identity(), epsilon);
  }

  // Products
  constexpr Quaternion<T> operator*(const Quaternion<T>& other) const {
    return {
        W() * other.W() - X() * other.X() - Y() * other.Y() - Z() * other.Z(),
        W() * other.X() + X() * other.W() + Y() * other.Z() - Z() * other.Y(),
        W() * other.Y() - X() * other.Z() + Y() * other.W() + Z() * other.X(),
        W() * other.Z() + X() * other.Y() - Y() * other.X() + Z() * other.W()
    };
  }

  constexpr Quaternion<T>& operator*=(const Quaternion<T>& other) {
    *this = *this * other;
    return *this;
  }

  // Norms
  constexpr T Norm() const { return data_.Norm(); }
  constexpr T SquaredNorm() const { return data_.NormSquared(); }
  constexpr Quaternion<T> Normalize() const {
    Quaternion<T> result = *this;
    return result.NormalizeInPlace();
  }
  constexpr Quaternion<T>& NormalizeInPlace() {
    data_.NormalizeInPlace();
    return *this;
  }

  // Quaternion Operations
  constexpr Quaternion<T> Conjugate() const {
    Quaternion<T> result = *this;
    return result.ConjugateInPlace();
  }
  constexpr Quaternion<T>& ConjugateInPlace() {
    data_[1] = -X();
    data_[2] = -Y();
    data_[3] = -Z();
    return *this;
  }
  constexpr Quaternion<T> Inverse() const {
    Quaternion<T> result = *this;
    return result.InverseInPlace();
  }
  constexpr Quaternion<T>& InverseInPlace() {
    T h2 = SquaredNorm();
    data_[0] /= h2;
    data_[1] /= -h2;
    data_[2] /= -h2;
    data_[3] /= -h2;
    return *this;
  }

  // Geometry
  constexpr Vector3<T> Rotate(const Vector3<T>& v) const {
    Vector3<T> qv(X(), Y(), Z());

    Vector3<T> t(
        T{2} * (qv.Y() * v.Z() - qv.Z() * v.Y()),
        T{2} * (qv.Z() * v.X() - qv.X() * v.Z()),
        T{2} * (qv.X() * v.Y() - qv.Y() * v.X())
    );

    return Vector3<T>(
        v.X() + W() * t.X() + (qv.Y() * t.Z() - qv.Z() * t.Y()),
        v.Y() + W() * t.Y() + (qv.Z() * t.X() - qv.X() * t.Z()),
        v.Z() + W() * t.Z() + (qv.X() * t.Y() - qv.Y() * t.X())
    );
  }

  // Interpolation
  static constexpr Quaternion<T> Slerp(
      const Quaternion<T>& a_in, const Quaternion<T>& b, T t
  ) {
    using std::abs;
    using std::acos;
    using std::cos;
    using std::sin;
    using xsimd::abs;
    using xsimd::acos;
    using xsimd::cos;
    using xsimd::sin;

    T kDotThreshold = T{1e-6F};
    T kEpsilon = T{1e-8F};

    T raw_dot = a_in.AsMatrix().Dot(b.AsMatrix());
    auto flip_mask = raw_dot < T{0.0};

    T dot = xsimd::select(flip_mask, -raw_dot, raw_dot);
    Quaternion<T> a(a_in.AsMatrix() * util::Select(flip_mask, T{-1}, T{1}));

    dot = xsimd::min(dot, T{1.0});

    Quaternion<T> nlerp_result(
        a.AsMatrix() + (b.AsMatrix() - a.AsMatrix()) * t
    );
    nlerp_result.NormalizeInPlace();

    T theta_0 = acos(dot);
    T theta = theta_0 * t;

    T sin_theta_0 = sin(theta_0);
    T sin_theta = sin(theta);

    T sin_theta_0_safe =
        xsimd::select(abs(sin_theta_0) < kEpsilon, kEpsilon, sin_theta_0);

    T s1 = cos(theta) - dot * sin_theta / sin_theta_0_safe;
    T s2 = sin_theta / sin_theta_0_safe;

    Quaternion<T> slerp_result(a.AsMatrix() * s1 + b.AsMatrix() * s2);

    auto near_mask = dot > (T{1.0} - kDotThreshold);
    return Quaternion<T>(
        xsimd::select(near_mask, nlerp_result.W(), slerp_result.W()),
        xsimd::select(near_mask, nlerp_result.X(), slerp_result.X()),
        xsimd::select(near_mask, nlerp_result.Y(), slerp_result.Y()),
        xsimd::select(near_mask, nlerp_result.Z(), slerp_result.Z())
    );
  }

  // Printing
  friend std::ostream& operator<<(
      std::ostream& os, const math::Quaternion<T>& q
  ) {
    os << "Quaternion(" << q.W() << ", " << q.X() << ", " << q.Y() << ", "
       << q.Z() << ")";
    return os;
  }

 private:
  Matrix<T, 4, 1> data_;
};

template <typename T>
using QuaternionAssembler =
    engine::Assembler<Quaternion, T, engine::RepeatTypes<4>>;
static_assert(engine::AssemblerLike<QuaternionAssembler<float>>);

}  // namespace achilles::domain::math
