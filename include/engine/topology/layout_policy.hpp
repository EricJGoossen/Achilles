#pragma once

#include <bit>
#include <cstddef>
#include <type_traits>
#include <xsimd/xsimd.hpp>

#include "util/simd_ops.hpp"

namespace achilles::engine::topology {

// The memory-style seam, and the single home for per-field SIMD sizing math
// that used to be duplicated between View::FieldStrideBytes/FieldZero and
// the test fixture that exercises it, where the two had to agree by hand.
// A policy answers, for one Assembler and an instance count, everything the
// allocator and the View need to place and address that field's block --
// and, through its leaf access below, everything an Assembler needs to
// reach one leaf in it: the layout is the only thing that knows how a
// value's leaves sit in memory (SoA here), an Assembler only what leaves a
// value has. A future non-planar style (e.g. AoSLayout) is a sibling
// struct with this same surface -- View itself has no notion of "the"
// layout, only whatever policy each field's own binding names (see
// engine/view/view.hpp).
struct PlanarLayout {
  // xsimd::batch<A::ScalarType>::size. NOT assumed equal across fields: a
  // field whose Assembler's ScalarType isn't float (e.g. an integer mask
  // storage type) can batch at a different width, which is why Layout pads
  // to the max LaneCount over every hosted field, not to float's. A field
  // whose leaf is a whole record (LeafAssembler<R>) has no lanes: 1.
  template <typename A>
    requires std::is_arithmetic_v<typename A::ScalarType>
  static constexpr std::size_t LaneCount() {
    return xsimd::batch<typename A::ScalarType>::size;
  }
  template <typename A>
  static constexpr std::size_t LaneCount() {
    return 1;
  }

  template <typename A>
    requires std::is_arithmetic_v<typename A::ScalarType>
  static constexpr std::size_t Alignment() {
    return alignof(xsimd::batch<typename A::ScalarType>);
  }
  template <typename A>
  static constexpr std::size_t Alignment() {
    return alignof(typename A::ScalarType);
  }

  // Per-instance byte step within one leaf's own flat array. One leaf
  // element wide under PlanarLayout, since each leaf is stored as a
  // contiguous run of ScalarType -- this is what makes "instance index *
  // this" the right per-row address step. A non-planar policy (e.g. one
  // interleaving a record's fields) would answer differently here.
  template <typename A>
  static constexpr std::size_t ElementStrideBytes() {
    return sizeof(typename A::ScalarType);
  }

  // Byte distance between field F's own sub-fields (e.g. x/y/z of a Vector3),
  // rounded up to Alignment<A>() so every leaf sub-array -- not just the
  // first -- starts batch-aligned.
  template <typename A>
  static constexpr std::size_t StrideBytes(std::size_t padded_instances) {
    std::size_t alignment = Alignment<A>();
    std::size_t raw_bytes = padded_instances * sizeof(typename A::ScalarType);
    return (raw_bytes + alignment - 1) / alignment * alignment;
  }

  // Total bytes for this field's block: one leaf-array of StrideBytes, times
  // however many leaves A::kNumFields flattens to.
  template <typename A>
  static constexpr std::size_t BlockBytes(std::size_t padded_instances) {
    return StrideBytes<A>(padded_instances) * A::kNumFields;
  }

  // padded_instances must be a multiple of LaneCount<A>() or
  // View::NumBatches's truncating division silently drops a tail batch.
  // Layout is responsible for guaranteeing this; this is the build-time
  // check that would catch it not holding.
  template <typename A>
  static constexpr bool IsLaneMultiple(std::size_t padded_instances) {
    return padded_instances % LaneCount<A>() == 0;
  }

  // -- Leaf access: everything an Assembler (engine/assembler.hpp) needs to
  // reach its leaves, so it never assumes a memory shape itself. `element`
  // is one instance's address in leaf 0 (the field's base plus that
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

// Kept intentionally thin: the real per-Assembler checks happen at the use
// site (SimAllocator instantiates BlockBytes<A>() for a concrete A), where a
// malformed policy fails with the offending A named, rather than here.
template <typename P>
concept LayoutPolicyLike = std::is_empty_v<P>;

}  // namespace achilles::engine::topology
