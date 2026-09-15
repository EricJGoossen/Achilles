#pragma once

#include <algorithm>
#include <array>
#include <cstddef>

#include "domain/math/activation_mask.hpp"

namespace achilles::test_support {

// MaskLike (domain/math/activation_mask.hpp) is a generic concept with two
// production consumers: Matrix::ApplyMaskRowsInPlace/ApplyMaskColsInPlace/
// MaskedInverseInPlace (domain/math/matrix.hpp) and
// InertiaOperator::MaskedInverse (domain/spatial/inertia.hpp), neither
// templated on ActivationMask specifically. MaskArchetype implements
// exactly what MaskLike requires and nothing more -- static_asserted below
// -- so tests that exercise those methods through it prove they rely only
// on the MaskLike interface, not on anything ActivationMask happens to
// also provide (its ScalarType, its bit-packed representation, ...).
// ArithmeticLike (util/simd_ops.hpp), the other concept this codebase's
// math/spatial headers use, gets no archetype: it's a closed type-category
// check (built-in arithmetic or an xsimd::batch of one) with no interface
// of its own to accidentally over-rely on, so there's nothing for an
// archetype to catch.
template <std::size_t N>
struct MaskArchetype {
  std::array<bool, N> bits{};

  static MaskArchetype Zero() { return MaskArchetype{}; }
  static MaskArchetype Ones() {
    MaskArchetype m;
    m.bits.fill(true);
    return m;
  }
  MaskArchetype& SetZero() {
    bits.fill(false);
    return *this;
  }
  MaskArchetype& SetOnes() {
    bits.fill(true);
    return *this;
  }
  MaskArchetype& NegateInPlace() {
    for (auto&& b : bits) {
      b = !b;
    }
    return *this;
  }

  bool operator[](std::size_t i) const { return bits.at(i); }
  int AsStorage() const {
    int result = 0;
    for (std::size_t i = 0; i < N; ++i) {
      result |= (bits[i] ? 1 : 0) << i;
    }
    return result;
  }
  static constexpr std::size_t Size() { return N; }

  bool operator==(const MaskArchetype& other) const {
    return bits == other.bits;
  }
  bool operator!=(const MaskArchetype& other) const {
    return !(*this == other);
  }
  bool AllTrue() const {
    return std::ranges::all_of(bits, [](bool b) { return b; });
  }
  bool AllFalse() const {
    return std::ranges::all_of(bits, [](bool b) { return !b; });
  }

  MaskArchetype operator&(const MaskArchetype& other) const {
    MaskArchetype result;
    for (std::size_t i = 0; i < N; ++i) {
      result.bits[i] = bits[i] && other.bits[i];
    }
    return result;
  }
  MaskArchetype& operator&=(const MaskArchetype& other) {
    return *this = *this & other;
  }
  MaskArchetype operator|(const MaskArchetype& other) const {
    MaskArchetype result;
    for (std::size_t i = 0; i < N; ++i) {
      result.bits[i] = bits[i] || other.bits[i];
    }
    return result;
  }
  MaskArchetype& operator|=(const MaskArchetype& other) {
    return *this = *this | other;
  }
  MaskArchetype operator^(const MaskArchetype& other) const {
    MaskArchetype result;
    for (std::size_t i = 0; i < N; ++i) {
      result.bits[i] = bits[i] != other.bits[i];
    }
    return result;
  }
  MaskArchetype& operator^=(const MaskArchetype& other) {
    return *this = *this ^ other;
  }
  MaskArchetype operator~() const {
    MaskArchetype result = *this;
    result.NegateInPlace();
    return result;
  }
};
static_assert(achilles::domain::math::MaskLike<MaskArchetype<3>>);

}  // namespace achilles::test_support
