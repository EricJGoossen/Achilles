// The base every layout policy derives from, and the layout concepts.

#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>
#include <vector>
#include <xsimd/xsimd.hpp>

#include "util/simd_ops.hpp"
#include "util/tmp.hpp"

namespace achilles::policy::layout {

// What every layout policy derives from: logic its layout policys share goes
// here. Never used polymorphically: nothing holds or calls one through it.
struct BaseLayout {
  // -- Leaf access: everything an Assembler (engine/assembly/assembler.hpp)
  // needs to reach its leaves, so it never assumes a memory shape itself.
  // `element` is one instance's address in leaf 0 (the field's base plus that
  // instance's ElementStrideBytes step); `stride` is StrideBytes.

  // Leaf `leaf` of the instance at `element`: its own planar array, `leaf`
  // strides further on.
  static constexpr const std::byte* LeafAt(
      const std::byte* element, std::size_t leaf, std::size_t stride
  ) {
    return element + (leaf * stride);
  }
  static constexpr std::byte* LeafAt(
      std::byte* element, std::size_t leaf, std::size_t stride
  ) {
    return element + (leaf * stride);
  }

  // One leaf, as TargetScalar: a plain Leaf, or -- the lanes of a leaf
  // being consecutive here -- an aligned load of a whole batch of them.
  template <typename TargetScalar, typename Leaf>
  static TargetScalar LoadLeaf(const std::byte* leaf) {
    return util::SimdLoad<TargetScalar>(std::bit_cast<const Leaf*>(leaf), 0);
  }

  // The one Leaf at `leaf`, in every lane of TargetScalar.
  template <typename TargetScalar, typename Leaf>
  static TargetScalar BroadcastLeaf(const std::byte* leaf) {
    return util::SimdBroadcast<TargetScalar>(std::bit_cast<const Leaf*>(leaf));
  }

  template <typename Leaf, typename ValueT>
  static void StoreLeaf(std::byte* leaf, const ValueT& value) {
    util::SimdStore(std::bit_cast<Leaf*>(leaf), 0, value);
  }

  // The one Leaf at `leaf` += the sum of value's lanes.
  template <typename Leaf, typename ValueT>
  static void AccumulateLeaf(std::byte* leaf, const ValueT& value) {
    util::SimdAccumulate(std::bit_cast<Leaf*>(leaf), value);
  }
};

// The RowTable of a layout that needs none (see PlanarLayout::RowTable).
struct NoRowTable {};

// A layout placing rows from a segment table (SegmentedLayout).
template <typename L>
concept SegmentedLayoutLike = requires {
  { L::kSegmented } -> std::convertible_to<bool>;
} && L::kSegmented;

// Kept intentionally thin: the real per-Assembler checks happen at the use
// site (SimAllocator instantiates BlockBytes<A>() for a concrete A), where a
// malformed policy fails with the offending A named, rather than here.
template <typename P>
concept LayoutPolicyLike = std::is_empty_v<P>;

}  // namespace achilles::policy::layout
