#pragma once

#include <array>
#include <cassert>
#include <cmath>
#include <complex>
#include <cstddef>
#include <type_traits>
#include <utility>

#include "domain/math/activation_mask.hpp"
#include "domain/math/matrix.hpp"
#include "domain/math/quaternion.hpp"
#include "domain/math/vector3.hpp"
#include "dual.hpp"
#include "engine/assembler.hpp"
#include "util/simd_ops.hpp"

namespace achilles::domain::spatial {

template <util::ArithmeticLike T, bool Inverted = false>
class InertiaOperator;

template <util::ArithmeticLike T>
class Inertia {
  constexpr static std::size_t kNumInertiaElements = 6;
  using Mask = decltype(std::declval<T>() == std::declval<T>());
  using Vector3 = math::Vector3<T>;
  using Matrix3x3 = math::Matrix3x3<T>;
  using Matrix6x6 = math::Matrix6x6<T>;

 public:
  using ScalarType = T;

  // Constructors
  inline constexpr Inertia() : mass_(), h_(), inertias_() {}
  constexpr Inertia(
      T mass, const Vector3& h, T Ixx, T Iyy, T Izz, T Ixy, T Ixz, T Iyz
  )
      : mass_(mass), h_(h), inertias_{Ixx, Ixy, Ixz, Iyy, Iyz, Izz} {
    assert(
        util::AllTrue(IsPhysicallyValid()) &&
        "Inertia parameters do not correspond to a physically realizable "
        "rigid body"
    );
  }

  inline constexpr Inertia(T mass, const Vector3& h, const Matrix3x3& i)
      : mass_(mass),
        h_(h),
        inertias_{i(0, 0), i(0, 1), i(0, 2), i(1, 1), i(1, 2), i(2, 2)} {
    assert(
        !util::AnyTrue(i(0, 1) != i(1, 0)) &&
        !util::AnyTrue(i(0, 2) != i(2, 0)) &&
        !util::AnyTrue(i(1, 2) != i(2, 1)) && "Inertia matrix must be symmetric"
    );
    assert(
        util::AllTrue(IsPhysicallyValid()) &&
        "Inertia parameters do not correspond to a physically realizable "
        "rigid body"
    );
  }

  // Static Constructors
  static constexpr Inertia Zero() { return {}; }
  constexpr Inertia& SetZero() {
    mass_ = T{0};
    h_ = Vector3::Zero();
    inertias_ = std::array<T, kNumInertiaElements>{};
    return *this;
  }
  static constexpr Inertia Identity() {
    return {T{1}, Vector3::Zero(), T{1}, T{1}, T{1}, T{0}, T{0}, T{0}};
  }
  constexpr Inertia& SetIdentity() {
    mass_ = T{1};
    h_ = Vector3::Zero();
    inertias_ =
        std::array<T, kNumInertiaElements>{T{1}, T{0}, T{0}, T{1}, T{0}, T{1}};
    return *this;
  }

  // Access
  inline constexpr std::tuple<T, Vector3, T, T, T, T, T, T> ToTuple() const {
    return std::make_tuple(
        Mass(), H(), Ixx(), Iyy(), Izz(), Ixy(), Ixz(), Iyz()
    );
  }
  inline constexpr T Mass() const { return mass_; }
  inline constexpr Vector3 H() const { return h_; }
  inline constexpr T Ixx() const { return inertias_[0]; }
  inline constexpr T Ixy() const { return inertias_[1]; }
  inline constexpr T Ixz() const { return inertias_[2]; }
  inline constexpr T Iyy() const { return inertias_[3]; }
  inline constexpr T Iyz() const { return inertias_[4]; }
  inline constexpr T Izz() const { return inertias_[5]; }
  inline constexpr Matrix6x6 AsMatrix() const {
    Matrix6x6 result;
    result.SetSubmatrix(0, 0, RotationalMatrix());
    result.SetSubmatrix(0, 3, h_.Skew());
    result.SetSubmatrix(3, 0, h_.Skew().TransposeInPlace());
    result.SetSubmatrix(3, 3, mass_ * Matrix3x3::Identity());
    return result;
  }
  inline constexpr Matrix3x3 RotationalMatrix() const {
    // clang-format off
        return {
            inertias_[0], inertias_[1], inertias_[2],
            inertias_[1], inertias_[3], inertias_[4],
            inertias_[2], inertias_[4], inertias_[5]
        };
    // clang-format on
  }
  inline constexpr InertiaOperator<T> AsArticulated() const {
    return AsMatrix();
  };

  // Comparison
  friend inline constexpr Mask operator==(const Inertia& a, const Inertia& b) {
    Mask result = (a.mass_ == b.mass_);
    result = result & (a.h_ == b.h_);
    for (std::size_t i = 0; i < kNumInertiaElements; i++) {
      result = result & (a.inertias_[i] == b.inertias_[i]);
    }
    return result;
  }
  friend inline constexpr Mask operator!=(const Inertia& a, const Inertia& b) {
    return !(a == b);
  }
  inline constexpr Mask IsApprox(const Inertia& other, float epsilon = 1e-5)
      const {
    using std::abs;
    using xsimd::abs;
    Mask result = abs(mass_ - other.mass_) <= epsilon;
    result = result & (h_.IsApprox(other.h_, epsilon));
    for (std::size_t i = 0; i < kNumInertiaElements; i++) {
      result = result & (abs(inertias_[i] - other.inertias_[i]) <= epsilon);
    }
    return result;
  }
  inline constexpr Mask IsZero(float epsilon = 1e-8) const {
    return this->IsApprox(Inertia::Zero(), epsilon);
  }

  // Elementwise Arithmetic
  inline constexpr Inertia operator+(const Inertia& other) const {
    Inertia result;
    for (std::size_t i = 0; i < kNumInertiaElements; i++) {
      result.inertias_[i] = inertias_[i] + other.inertias_[i];
    }
    result.mass_ = mass_ + other.mass_;
    result.h_ = h_ + other.h_;
    return result;
  }
  inline constexpr InertiaOperator<T> operator+(const InertiaOperator<T>& other
  ) const {
    return this->AsArticulated() += other;
  }
  inline constexpr Inertia operator-(const Inertia& other) const {
    Inertia result;
    for (std::size_t i = 0; i < kNumInertiaElements; i++) {
      result.inertias_[i] = inertias_[i] - other.inertias_[i];
    }
    result.mass_ = mass_ - other.mass_;
    result.h_ = h_ - other.h_;

    assert(util::AllTrue(result.mass_ >= T{0}));
    return result;
  }
  inline constexpr InertiaOperator<T> operator-(const InertiaOperator<T>& other
  ) const {
    return this->AsArticulated() -= other;
  }
  inline constexpr Inertia& operator+=(const Inertia& other) {
    mass_ += other.mass_;
    h_ += other.h_;
    for (std::size_t i = 0; i < kNumInertiaElements; i++) {
      inertias_[i] += other.inertias_[i];
    }
    return *this;
  }
  inline constexpr Inertia& operator-=(const Inertia& other) {
    mass_ -= other.mass_;
    h_ -= other.h_;
    for (std::size_t i = 0; i < kNumInertiaElements; i++) {
      inertias_[i] -= other.inertias_[i];
    }
    assert(mass_ >= T{0});
    return *this;
  }

  // Scalar Algebra
  inline constexpr Inertia operator*(T scalar) const {
    assert(scalar > T{0});
    Inertia result;
    for (std::size_t i = 0; i < kNumInertiaElements; i++) {
      result.inertias_[i] = inertias_[i] * scalar;
    }
    result.mass_ = mass_ * scalar;
    result.h_ = h_ * scalar;
    return result;
  }
  inline constexpr Inertia operator/(T scalar) const {
    assert(scalar > T{0});
    Inertia result;
    for (std::size_t i = 0; i < kNumInertiaElements; i++) {
      result.inertias_[i] = inertias_[i] / scalar;
    }
    result.mass_ = mass_ / scalar;
    result.h_ = h_ / scalar;
    return result;
  }
  inline constexpr Inertia& operator*=(T scalar) {
    assert(scalar > T{0});
    for (std::size_t i = 0; i < kNumInertiaElements; i++) {
      inertias_[i] *= scalar;
    }
    mass_ *= scalar;
    h_ *= scalar;
    return *this;
  }
  inline constexpr Inertia& operator/=(T scalar) {
    assert(scalar > T{0});
    for (std::size_t i = 0; i < kNumInertiaElements; i++) {
      inertias_[i] /= scalar;
    }
    mass_ /= scalar;
    h_ /= scalar;
    return *this;
  }

  // Inertia Operations
  inline constexpr SpatialMomentum<T> Apply(const SpatialVelocity<T>& v) const {
    Vector3 f = mass_ * v.Linear() + v.Angular().Cross(h_);
    Vector3 tau = RotationalMatrix() * v.Angular() + h_.Cross(v.Linear());
    return {tau, f};
  }
  inline constexpr SpatialForce<T> Apply(const SpatialAcceleration<T>& v
  ) const {
    Vector3 f = mass_ * v.Linear() + v.Angular().Cross(h_);
    Vector3 tau = RotationalMatrix() * v.Angular() + h_.Cross(v.Linear());
    return {tau, f};
  }
  inline constexpr InertiaOperator<T, true> Inverse() const {
    assert(mass_ > T{0});

    Matrix6x6 mat;
    Matrix3x3 S = h_.Skew();
    T m_inv = T{1} / mass_;

    math::MatrixBlock<T, 3, 3> I_com_inv = mat.template Block<3, 3>(0, 0);
    I_com_inv = (RotationalMatrix() + m_inv * (S * S)).InverseInPlace();
    Matrix3x3 P_S = I_com_inv * S;

    mat.template Block<3, 3>(0, 3) = (T{-1} * m_inv) * P_S;
    mat.template Block<3, 3>(3, 0) = (T{-1} * m_inv) * P_S.Transpose();
    mat.template Block<3, 3>(3, 3) =
        m_inv * Matrix3x3::Identity() - (m_inv * m_inv) * (S * P_S);

    return mat;
  }

  // Printing
  friend std::ostream& operator<<(
      std::ostream& os, const spatial::Inertia<T>& i
  ) {
    os << "Inertia(" << i.AsMatrix() << ")";
    return os;
  }

 private:
  static constexpr std::array<T, 3> SymmetricEigenvalues(const Matrix3x3& A) {
    using std::sqrt, std::acos, std::cos, std::min, std::max;
    using xsimd::sqrt, xsimd::acos, xsimd::cos, xsimd::min, xsimd::max;

    T p1 = A(0, 1) * A(0, 1) + A(0, 2) * A(0, 2) + A(1, 2) * A(1, 2);
    T trace = A(0, 0) + A(1, 1) + A(2, 2);
    T q = trace / T{3};
    T p2 = (A(0, 0) - q) * (A(0, 0) - q) + (A(1, 1) - q) * (A(1, 1) - q) +
           (A(2, 2) - q) * (A(2, 2) - q) + T{2} * p1;
    T p = sqrt(p2 / T{6});

    // p is 0 exactly when A is isotropic (a scalar multiple of Identity,
    // i.e. all three eigenvalues already equal q) -- the trig formula
    // below divides by p and is 0/0-indeterminate there, producing NaN
    // instead of the q it should. Substitute a safe nonzero divisor for
    // just the division (its result is discarded below wherever the mask
    // says isotropic) rather than branching, so this stays correct
    // per-lane for a batched T the same way the rest of this function is.
    auto isotropic = p2 <= T{1e-10};
    T safe_p = util::Select(isotropic, T{1}, p);
    Matrix3x3 B = (T{1} / safe_p) * (A - q * Matrix3x3::Identity());
    T r = min(max(B.Determinant() / T{2}, T{-1}), T{1});
    T phi = acos(r) / T{3};

    T eig3_raw = q + T{2} * p * cos(phi);
    T eig1_raw = q + T{2} * p * cos(phi + T{2 * M_PI / 3});
    T eig3 = util::Select(isotropic, q, eig3_raw);
    T eig1 = util::Select(isotropic, q, eig1_raw);
    T eig2 = trace - eig1 - eig3;

    // A per-lane batch has no single sort order, so this can't use
    // std::sort -- instead it's a branchless 3-element sorting network
    // (min/max only) that sorts each lane independently.
    T lo = min(eig1, eig2);
    T hi = max(eig1, eig2);
    T sorted_hi = max(hi, eig3);
    T mid_candidate = min(hi, eig3);
    T sorted_lo = min(lo, mid_candidate);
    T sorted_mid = max(lo, mid_candidate);

    return {sorted_lo, sorted_mid, sorted_hi};
  }

  inline constexpr Mask IsPhysicallyValid(float epsilon = 1e-8) const {
    Mask valid = (mass_ > T{0});
    Matrix3x3 I_com =
        RotationalMatrix() + (T{1} / mass_) * (h_.Skew() * h_.Skew());
    auto eig =
        SymmetricEigenvalues(I_com);  // ascending: eig[0] <= eig[1] <= eig[2]
    valid = valid & (eig[0] >= T{-epsilon});  // PSD
    valid = valid &
            (eig[0] + eig[1] >= eig[2] - T{epsilon});  // triangle inequality
    return valid;
  }

  T mass_;
  Vector3 h_;
  std::array<T, kNumInertiaElements> inertias_;
};

// Field order matches the (mass, h, Ixx, Iyy, Izz, Ixy, Ixz, Iyz)
// constructor -- Inertia::ToTuple() must agree with it for Read/Write to
// round-trip through this Assembler correctly.
template <typename T>
using InertiaAssembler = engine::
    Assembler<Inertia, T, T, math::Vector3Assembler<T>, engine::RepeatTypes<6>>;
static_assert(engine::AssemblerLike<InertiaAssembler<float>>);

template <util::ArithmeticLike T, bool Inverted>
class InertiaOperator {
  using Mask = decltype(std::declval<T>() == std::declval<T>());

  using Matrix6x6 = math::Matrix6x6<T>;

 public:
  using ScalarType = T;

  // Constructors
  inline constexpr InertiaOperator() : data_() {}
  inline constexpr InertiaOperator(const Matrix6x6& data) : data_(data) {}

  // Static Constructors
  static constexpr InertiaOperator Zero() { return {}; }
  constexpr InertiaOperator& SetZero() {
    data_.SetZero();
    return *this;
  }
  static constexpr InertiaOperator Identity() { return Matrix6x6::Identity(); }
  constexpr InertiaOperator& SetIdentity() {
    data_.SetIdentity();
    return *this;
  }

  // Access
  inline constexpr std::tuple<Matrix6x6> ToTuple() const {
    return std::make_tuple(data_);
  }
  constexpr Matrix6x6 AsMatrix() const { return data_; }
  constexpr bool IsInverse() const { return Inverted; }
  constexpr Inertia<T> AsSparse() const {
    static_assert(!Inverted, "Cannot convert InverseInertia to Inertia");
    assert(
        util::AllTrue(IsSparseRepresentable()) &&
        "InertiaOperator does not have rigid-body-inertia block "
        "structure "
        " and cannot be converted to Inertia"
    );
    return {
        data_(3, 3),
        {data_(2, 4), data_(0, 5), data_(1, 3)},
        data_(0, 0),
        data_(1, 1),
        data_(2, 2),
        data_(0, 1),
        data_(0, 2),
        data_(1, 2)
    };
  }

  // Comparison
  friend inline constexpr Mask operator==(
      const InertiaOperator& a, const InertiaOperator& b
  ) {
    return a.data_ == b.data_;
  }
  friend inline constexpr Mask operator!=(
      const InertiaOperator& a, const InertiaOperator& b
  ) {
    return a.data_ != b.data_;
  }
  inline constexpr Mask IsApprox(
      const InertiaOperator& other, float epsilon = 1e-5
  ) const {
    return data_.IsApprox(other.data_, epsilon);
  }
  inline constexpr Mask IsZero(float epsilon = 1e-8) const {
    return data_.IsZero(epsilon);
  }

  // Elementwise Arithmetic
  inline constexpr InertiaOperator operator+(const InertiaOperator& other
  ) const {
    return data_ + other.data_;
  }
  inline constexpr InertiaOperator operator+(const Inertia<T>& other) const {
    static_assert(!Inverted, "Cannot add Inertia to InverseInertia");
    return other.AsArticulated() += *this;
  }
  inline constexpr InertiaOperator operator-(const InertiaOperator& other
  ) const {
    return data_ - other.data_;
  }
  inline constexpr InertiaOperator operator-(const Inertia<T>& other) const {
    static_assert(!Inverted, "Cannot subtract Inertia from InverseInertia");
    return data_ - other.AsArticulated().AsMatrix();
  }
  inline constexpr InertiaOperator& operator+=(const InertiaOperator& other) {
    data_ += other.data_;
    return *this;
  }
  inline constexpr InertiaOperator& operator+=(const Inertia<T>& other) {
    static_assert(!Inverted, "Cannot add Inertia to InverseInertia");
    return *this += other.AsArticulated();
  }
  inline constexpr InertiaOperator& operator-=(const InertiaOperator& other) {
    data_ -= other.data_;
    return *this;
  }
  inline constexpr InertiaOperator& operator-=(const Inertia<T>& other) {
    static_assert(!Inverted, "Cannot subtract Inertia from InverseInertia");
    return *this -= other.AsArticulated();
  }

  // Scalar Algebra
  inline constexpr InertiaOperator operator*(T scalar) const {
    return data_ * scalar;
  }
  inline constexpr InertiaOperator operator/(T scalar) const {
    return data_ / scalar;
  }
  inline constexpr InertiaOperator& operator*=(T scalar) {
    data_ *= scalar;
    return *this;
  }
  inline constexpr InertiaOperator& operator/=(T scalar) {
    data_ /= scalar;
    return *this;
  }

  // Inertia Operations - these are not valid for inverse inertia
  inline constexpr SpatialMomentum<T> Apply(const SpatialVelocity<T>& v) const {
    static_assert(!Inverted, "Cannot apply InverseInertia to SpatialVelocity");
    return (data_ * v).template As<SpatialMomentum>();
  }
  inline constexpr SpatialForce<T> Apply(const SpatialAcceleration<T>& v
  ) const {
    static_assert(
        !Inverted, "Cannot apply InverseInertia to SpatialAcceleration"
    );
    return (data_ * v).template As<SpatialForce>();
  }

  // Inertia Operations - these are only valid for inverse inertia
  inline constexpr SpatialVelocity<T> Apply(const SpatialMomentum<T>& v) const {
    static_assert(Inverted, "Cannot apply InertiaOperator to SpatialMomentum");
    return (data_ * v).template As<SpatialVelocity>();
  }
  inline constexpr SpatialAcceleration<T> Apply(const SpatialForce<T>& v
  ) const {
    static_assert(
        Inverted, "Cannot apply InertiaOperator to SpatialAcceleration"
    );
    return (data_ * v).template As<SpatialAcceleration>();
  }

  // Matrix Operations
  inline constexpr InertiaOperator operator*(const InertiaOperator& other
  ) const {
    return {data_ * other.data_};
  }
  inline constexpr InertiaOperator operator*(const Matrix6x6& m) const {
    return {data_ * m};
  }
  friend inline constexpr InertiaOperator operator*(
      const Matrix6x6& m, const InertiaOperator& d
  ) {
    return {m * d.data_};
  }
  inline constexpr InertiaOperator Transpose() const {
    return {data_.Transpose()};
  }
  inline constexpr InertiaOperator& TransposeInPlace() {
    data_.TransposeInPlace();
    return *this;
  }
  inline constexpr InertiaOperator<T, !Inverted> Inverse() const {
    return {data_.Inverse()};
  }
  template <math::MaskLike MaskT>
    requires(MaskT::Size() == 6)
  inline constexpr InertiaOperator<T, !Inverted> MaskedInverse(const MaskT& m
  ) const {
    return {data_.MaskedInverse(m)};
  }

 private:
  inline constexpr Mask IsSparseRepresentable(float epsilon = 1e-6) const {
    using std::abs;
    using xsimd::abs;

    // Bottom-right block must be m*Identity for some scalar m.
    T m = data_(3, 3);
    Mask valid = (m > T{0});
    valid = valid & (abs(data_(4, 4) - m) <= epsilon);
    valid = valid & (abs(data_(5, 5) - m) <= epsilon);
    valid = valid & (abs(data_(3, 4)) <= epsilon);
    valid = valid & (abs(data_(3, 5)) <= epsilon);
    valid = valid & (abs(data_(4, 5)) <= epsilon);

    // Top-left block (I_o) must be symmetric.
    valid = valid & (abs(data_(0, 1) - data_(1, 0)) <= epsilon);
    valid = valid & (abs(data_(0, 2) - data_(2, 0)) <= epsilon);
    valid = valid & (abs(data_(1, 2) - data_(2, 1)) <= epsilon);

    // Top-right block must be skew-symmetric (diag zero, antisymmetric).
    valid = valid & (abs(data_(0, 3)) <= epsilon);
    valid = valid & (abs(data_(1, 4)) <= epsilon);
    valid = valid & (abs(data_(2, 5)) <= epsilon);
    valid = valid & (abs(data_(0, 4) + data_(1, 3)) <= epsilon);
    valid = valid & (abs(data_(0, 5) + data_(2, 3)) <= epsilon);
    valid = valid & (abs(data_(1, 5) + data_(2, 4)) <= epsilon);

    // Bottom-left block must equal the transpose of the top-right block.
    valid = valid & (abs(data_(3, 0) - data_(0, 3)) <= epsilon);
    valid = valid & (abs(data_(3, 1) - data_(1, 3)) <= epsilon);
    valid = valid & (abs(data_(3, 2) - data_(2, 3)) <= epsilon);
    valid = valid & (abs(data_(4, 0) - data_(0, 4)) <= epsilon);
    valid = valid & (abs(data_(4, 1) - data_(1, 4)) <= epsilon);
    valid = valid & (abs(data_(4, 2) - data_(2, 4)) <= epsilon);
    valid = valid & (abs(data_(5, 0) - data_(0, 5)) <= epsilon);
    valid = valid & (abs(data_(5, 1) - data_(1, 5)) <= epsilon);
    valid = valid & (abs(data_(5, 2) - data_(2, 5)) <= epsilon);

    return valid;
  }

  Matrix6x6 data_;
};

// InertiaOperator<T, Inverted> takes two template parameters, but
// Assembler's Target needs a single-scalar-parameter template -- these
// alias templates curry Inverted so each shape has its own Assembler.
template <typename T>
using ArticulatedInertiaOperator = InertiaOperator<T, false>;
template <typename T>
using InvertedArticulatedInertiaOperator = InertiaOperator<T, true>;

template <typename T>
using InertiaOperatorAssembler = engine::
    Assembler<ArticulatedInertiaOperator, T, math::Matrix6x6Assembler<T>>;
static_assert(engine::AssemblerLike<InertiaOperatorAssembler<float>>);
template <typename T>
using InvertedInertiaOperatorAssembler = engine::Assembler<
    InvertedArticulatedInertiaOperator,
    T,
    math::Matrix6x6Assembler<T>>;
static_assert(engine::AssemblerLike<InvertedInertiaOperatorAssembler<float>>);

}  // namespace achilles::domain::spatial
