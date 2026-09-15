#pragma once

#include <array>
#include <cstddef>
#include <tuple>
#include <xsimd/xsimd.hpp>

#include "engine/assembler.hpp"
#include "util/simd_ops.hpp"

namespace achilles::domain::math {

template <util::StorageLike StorageT, std::size_t N>
  requires(N > 0 && N <= sizeof(util::StorageLaneT<StorageT>) * 8)
class ActivationMask {
  using BatchMask = std::conditional_t<
      util::kIsXsimdBatch<StorageT>,
      xsimd::batch_bool<util::StorageLaneT<StorageT>>,
      bool>;
  static constexpr util::StorageLaneT<StorageT> kValidMask = (1 << N) - 1;

 public:
  using ScalarType = StorageT;

  // Constructors
  constexpr ActivationMask() : m_() {}
  constexpr ActivationMask(StorageT m) : m_(m & kValidMask) {}
  constexpr ActivationMask(std::array<bool, N> values) : m_(0) {
    for (std::size_t i = 0; i < N; i++) {
      m_ |= static_cast<StorageT>(values[i]) << i;
    }
  }
  template <typename... Bools>
    requires(sizeof...(Bools) == N && (std::convertible_to<Bools, bool> && ...))
  constexpr ActivationMask(Bools... values) : m_(0) {
    std::array<bool, N> values_array = {static_cast<bool>(values)...};

    for (std::size_t i = 0; i < N; i++) {
      m_ |= static_cast<StorageT>(values_array[i]) << i;
    }
  }

  // Static constructors
  static constexpr ActivationMask Zero() { return {}; }
  constexpr ActivationMask& SetZero() {
    m_ = 0;
    return *this;
  }

  static constexpr ActivationMask Ones() { return StorageT(kValidMask); }
  constexpr ActivationMask& SetOnes() {
    m_ = kValidMask;
    return *this;
  }

  // Access
  inline constexpr std::tuple<StorageT> ToTuple() const {
    return std::make_tuple(m_);
  }
  inline constexpr BatchMask operator[](std::size_t i) const {
    return (m_ & (1 << i)) != 0;
  }
  inline constexpr StorageT AsStorage() const { return m_; }
  inline static constexpr std::size_t Size() { return N; }
  inline constexpr bool IsBatched() const {
    return util::kIsXsimdBatch<StorageT>;
  }

  // Comparison
  friend inline constexpr BatchMask operator==(
      const ActivationMask& a, const ActivationMask& b
  ) {
    return (a.m_ == b.m_);
  }
  friend inline constexpr BatchMask operator!=(
      const ActivationMask& a, const ActivationMask& b
  ) {
    return !(a == b);
  }
  inline constexpr BatchMask AllTrue() const { return m_ == kValidMask; }
  inline constexpr BatchMask AllFalse() const { return m_ == 0; }

  // Elementwise Operations
  inline constexpr ActivationMask operator&(const ActivationMask& other) const {
    return m_ & other.m_;
  }
  inline constexpr ActivationMask& operator&=(const ActivationMask& other) {
    m_ &= other.m_;
    return *this;
  }
  inline constexpr ActivationMask operator|(const ActivationMask& other) const {
    return m_ | other.m_;
  }
  inline constexpr ActivationMask& operator|=(const ActivationMask& other) {
    m_ |= other.m_;
    return *this;
  }
  inline constexpr ActivationMask operator^(const ActivationMask& other) const {
    return m_ ^ other.m_;
  }
  inline constexpr ActivationMask& operator^=(const ActivationMask& other) {
    m_ ^= other.m_;
    return *this;
  }
  inline constexpr ActivationMask operator~() const { return ~m_ & kValidMask; }
  inline constexpr ActivationMask& NegateInPlace() {
    m_ = ~m_ & kValidMask;
    return *this;
  }

 private:
  StorageT m_;
};

template <typename T>
concept MaskLike = requires(T a, T ca, const T& other, std::size_t i) {
  // Static constructors
  { T::Zero() } -> std::same_as<T>;
  { T::Ones() } -> std::same_as<T>;

  // Mutating operations
  { a.SetZero() } -> std::same_as<T&>;
  { a.SetOnes() } -> std::same_as<T&>;
  { a.NegateInPlace() } -> std::same_as<T&>;

  // Access
  { ca[i] };
  { ca.AsStorage() };
  { T::Size() } -> std::same_as<std::size_t>;
  typename std::integral_constant<std::size_t, T::Size()>;

  // Comparison
  { ca == other };
  { ca != other };
  { ca.AllTrue() };
  { ca.AllFalse() };

  // Bitwise operations
  { ca& other } -> std::same_as<T>;
  { a &= other } -> std::same_as<T&>;
  { ca | other } -> std::same_as<T>;
  { a |= other } -> std::same_as<T&>;
  { ca ^ other } -> std::same_as<T>;
  { a ^= other } -> std::same_as<T&>;
  { ~ca } -> std::same_as<T>;
};
static_assert(MaskLike<ActivationMask<std::int32_t, 8>>);

// ActivationMask<StorageT, N> takes two template parameters, but
// Assembler's Target needs a single-scalar-parameter template -- this
// curries N so each width has its own Assembler.
template <std::size_t N>
struct ActivationMaskFor {
  template <typename StorageT>
  using Target = ActivationMask<StorageT, N>;
};

template <std::size_t N>
using ActivationMaskAssembler = engine::Assembler<
    ActivationMaskFor<N>::template Target,
    std::int32_t,
    engine::RepeatTypes<1>>;
static_assert(engine::AssemblerLike<ActivationMaskAssembler<8>>);

}  // namespace achilles::domain::math
