#pragma once

#include <cassert>
#include <cstddef>
#include <type_traits>
#include <xsimd/xsimd.hpp>

namespace achilles::util {

// Mask helpers: reduce a SIMD or scalar bool-like value to a single bool.
template <typename T>
  requires(xsimd::is_batch_bool<T>::value)
inline bool AnyTrue(const T& mask) {
  return xsimd::any(mask);
}
template <typename T>
inline bool AnyTrue(const T& mask) {
  return mask;
}

template <typename T>
  requires(xsimd::is_batch_bool<T>::value)
inline bool AllTrue(const T& mask) {
  return xsimd::all(mask);
}
template <typename T>
inline bool AllTrue(const T& mask) {
  return mask;
}

// Load/store T (scalar or xsimd::batch) at ptr[idx], dispatched at compile
// time. Batch overloads require idx aligned to the batch width.
template <typename T, typename ElemT>
  requires(xsimd::is_batch<T>::value)
inline T SimdLoad(const ElemT* ptr, std::size_t idx) {
  assert(
      idx % T::size == 0 && "SIMD load index must be aligned to batch width"
  );
  return xsimd::load_aligned(ptr + idx);
}
template <typename T, typename ElemT>
inline T SimdLoad(const ElemT* ptr, std::size_t idx) {
  return ptr[idx];
}

template <typename T, typename ElemT>
  requires(xsimd::is_batch<T>::value)
inline void SimdStore(ElemT* ptr, std::size_t idx, T val) {
  assert(
      idx % T::size == 0 && "SIMD store index must be aligned to batch width"
  );
  xsimd::store_aligned(ptr + idx, val);
}
template <typename T, typename ElemT>
inline void SimdStore(ElemT* ptr, std::size_t idx, T val) {
  ptr[idx] = val;
}

template <bool IsBatch, typename T>
using SimdT = std::conditional_t<IsBatch, xsimd::batch<T>, T>;

// Select between two values based on a scalar or SIMD mask. When the mask's
// lane type differs from T's (e.g. an integer activation mask selecting
// between floating-point values), the mask is bitwise-reinterpreted into T's
// element type before selecting.
template <typename MaskT, typename T>
  requires(xsimd::is_batch_bool<MaskT>::value && xsimd::is_batch<T>::value)
inline T Select(const MaskT& mask, T if_true, T if_false) {
  using ElemT = typename T::value_type;
  using MaskElemT = typename MaskT::value_type;
  if constexpr (std::is_same_v<MaskElemT, ElemT>) {
    return xsimd::select(mask, if_true, if_false);
  } else {
    auto elem_mask = xsimd::bitwise_cast<ElemT>(xsimd::bitwise_cast(mask)) !=
                     T{static_cast<ElemT>(0)};
    return xsimd::select(elem_mask, if_true, if_false);
  }
}
template <typename MaskT, typename T>
inline T Select(const MaskT& mask, T if_true, T if_false) {
  return mask ? if_true : if_false;
}

template <typename T>
struct IsXsimdBatch : std::false_type {};

template <typename T, typename Arch>
struct IsXsimdBatch<xsimd::batch<T, Arch>> : std::true_type {};

template <typename T>
inline constexpr bool kIsXsimdBatch = IsXsimdBatch<T>::value;

// How many scalars T packs per lane: 1 for a plain arithmetic T, or its
// xsimd batch width for an xsimd::batch<...> T. Lets code that indexes by
// "however many raw elements T covers" (e.g. planar storage strides) stay
// the same whether T is scalar or batched.
template <typename T>
constexpr std::size_t LaneCountOf() {
  if constexpr (kIsXsimdBatch<T>) {
    return T::size;
  } else {
    return 1;
  }
}

// A usable scalar throughout this codebase's math/spatial types: a plain
// arithmetic type (float, double, std::int32_t, ...) or an xsimd::batch
// of one. Every Matrix/Vector/Quaternion/Dual/Inertia type is written to
// work either way through its single scalar parameter T -- this is that
// requirement made explicit and checkable.
template <typename T>
concept ArithmeticLike =
    std::is_arithmetic_v<T> ||
    (kIsXsimdBatch<T> && std::is_arithmetic_v<typename T::value_type>);

// The int32 lane storage that pairs with some other scalar-or-batch type:
// plain std::int32_t for a scalar type, or a matching-width
// xsimd::batch<std::int32_t> for a batched one. Lets a bitmask-style type
// (e.g. ActivationMask) track another type's scalar/batch-ness (e.g. the T
// an Assembler is reading) without itself being templated on that
// unrelated type.
template <typename ScalarOrBatch>
using MaskStorageFor = std::conditional_t<
    kIsXsimdBatch<ScalarOrBatch>,
    xsimd::batch<std::int32_t>,
    std::int32_t>;

// A scalar-or-batch type restricted to integer lanes: plain std::int32_t,
// std::uint8_t, etc., or an xsimd::batch of one. Narrower than
// ArithmeticLike -- for code doing bitwise/shift operations (masks,
// bitfields) that are only meaningful on integers, not float/double.
template <typename StorageT>
concept StorageLike = std::is_integral_v<StorageT> ||
                      (kIsXsimdBatch<StorageT> &&
                       std::is_integral_v<typename StorageT::value_type>);

// The lane type of a StorageLike T: T itself for a scalar, or T's
// value_type for an xsimd batch. Lets code compute per-lane properties
// (e.g. bit width via sizeof(StorageLaneT<T>) * 8) uniformly whether T is
// scalar or batched.
template <typename T>
struct StorageLane {
  using Type = T;
};

template <typename T>
  requires kIsXsimdBatch<T>
struct StorageLane<T> {
  using Type = typename T::value_type;
};

template <typename T>
using StorageLaneT = typename StorageLane<T>::Type;

}  // namespace achilles::util