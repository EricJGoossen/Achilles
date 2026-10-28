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

namespace achilles::engine::topology {

// The RowTable of a layout that needs none (see PlanarLayout::RowTable).
struct NoRowTable {};

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

  // -- Row addressing through a field's RowTable: what a layout whose rows
  // aren't one fixed stride apart (SegmentedLayout below) places them
  // from. SimAllocator builds it from the plan (Segments), and a View over
  // the field holds it by value. A planar field's RowTable is empty -- it
  // takes no space in a View -- and these are exactly the fixed-stride
  // answers above.
  template <typename A>
  using RowTable = NoRowTable;

  template <typename A>
  static std::vector<std::size_t> Segments(
      std::span<const std::size_t> /*boundaries*/
  ) {
    return {};
  }

  template <typename A>
  static RowTable<A> RowTableOf(const std::vector<std::size_t>& /*segments*/) {
    return {};
  }

  // Byte offset of `row`, read as T, within one block.
  template <typename A, typename T>
  static constexpr std::size_t RowOffset(
      std::size_t row, NoRowTable /*table*/
  ) {
    return row * ElementStrideBytes<A>();
  }

  template <typename A>
  static constexpr std::size_t BlockBytes(
      std::size_t padded_instances, NoRowTable /*table*/
  ) {
    return BlockBytes<A>(padded_instances);
  }

  // What a View's cursor precomputes once per pass, so its per-row address
  // is one step from a base: nothing for a planar field, whose base is the
  // field's own.
  template <typename A>
  using CursorTable = NoRowTable;

  template <typename A>
  static CursorTable<A> CursorTableOf(
      std::byte* /*field0*/, NoRowTable /*table*/
  ) {
    return {};
  }

  template <typename A, typename T>
  static std::byte* CursorElement(
      std::byte* field0, const CursorTable<A>& /*table*/, std::size_t row
  ) {
    return field0 + RowOffset<A, T>(row, NoRowTable{});
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

namespace detail {

template <typename List>
struct RecordList;
template <typename... Rs>
struct RecordList<util::TypeList<Rs...>> {
  static constexpr std::size_t kCount = sizeof...(Rs);
  static constexpr std::array<std::size_t, kCount> kSizes = {sizeof(Rs)...};
  static constexpr std::array<std::size_t, kCount> kAlignments = {alignof(Rs)...
  };
  static constexpr std::size_t kMaxAlignment = std::max({alignof(Rs)...});
};

}  // namespace detail

// One record per row, of a different type per segment of the field's
// ordering: segment k of its topology (domain::SegmentedTopology)
// holds A::Records' k-th type, packed at that type's own size -- so a list
// mixing small and large records (a sensor list with encoders and a lidar)
// takes exactly what each row needs, and a pass reaches segment k's rows as
// that type (SegmentTraversal, with an Op written for it). A is a
// LeafAssembler over several records (engine/assembler.hpp). Padding rows
// sit in no segment and take no space.
//
// The field's RowTable is its segment table (Segments, built from the
// plan's segment boundaries), held by value: for each segment, its first
// row, then each segment's byte offset within the block, then the block's
// total bytes -- [first_0 .. first_{K-1}, offset_0 .. offset_{K-1},
// total]. Leaf access is planar's: a record is one leaf, read and written
// whole.
struct SegmentedLayout : PlanarLayout {
  static constexpr bool kSegmented = true;

  template <typename A>
  using RowTable = std::array<
      std::size_t,
      (2 * detail::RecordList<typename A::Records>::kCount) + 1>;

  template <typename A>
  static constexpr std::size_t LaneCount() {
    return 1;
  }

  template <typename A>
  static constexpr std::size_t Alignment() {
    return detail::RecordList<typename A::Records>::kMaxAlignment;
  }

  // One leaf per record: no sub-arrays to step between.
  template <typename A>
  static constexpr std::size_t StrideBytes(std::size_t /*padded_instances*/) {
    return 0;
  }

  // `boundaries`: segment k is rows [boundaries[k], boundaries[k + 1]).
  template <typename A>
  static std::vector<std::size_t> Segments(
      std::span<const std::size_t> boundaries
  ) {
    using List = detail::RecordList<typename A::Records>;
    constexpr std::size_t kCount = List::kCount;
    assert(
        boundaries.size() == kCount + 1 &&
        "SegmentedLayout: the field's ordering must have one segment per "
        "record type."
    );
    std::vector<std::size_t> table((2 * kCount) + 1);
    std::size_t offset = 0;
    for (std::size_t k = 0; k < kCount; ++k) {
      offset = RoundUp(offset, List::kAlignments[k]);
      table[k] = boundaries[k];
      table[kCount + k] = offset;
      offset += (boundaries[k + 1] - boundaries[k]) * List::kSizes[k];
    }
    // So a double-buffered field's second half starts aligned too.
    table[2 * kCount] = RoundUp(offset, List::kMaxAlignment);
    return table;
  }

  template <typename A>
  static RowTable<A> RowTableOf(const std::vector<std::size_t>& segments) {
    RowTable<A> table{};
    std::ranges::copy(segments, table.begin());
    return table;
  }

  // Per segment k, the address row 0 would have if all of segment k's
  // record type started at its own first row: field0 + offset_k -
  // first_k * sizeof(R_k). Held as integers -- for k past segment 0 it can
  // point before the block. With it, a row's address is one step from a
  // base, exactly a planar field's arithmetic.
  template <typename A>
  using CursorTable = std::
      array<std::uintptr_t, detail::RecordList<typename A::Records>::kCount>;

  template <typename A>
  static CursorTable<A> CursorTableOf(
      std::byte* field0, const RowTable<A>& segments
  ) {
    using List = detail::RecordList<typename A::Records>;
    CursorTable<A> bases{};
    const auto base = std::bit_cast<std::uintptr_t>(field0);
    for (std::size_t k = 0; k < List::kCount; ++k) {
      bases[k] =
          base + segments[List::kCount + k] - (segments[k] * List::kSizes[k]);
    }
    return bases;
  }

  template <typename A, typename T>
  static std::byte* CursorElement(
      std::byte* /*field0*/, const CursorTable<A>& bases, std::size_t row
  ) {
    return std::bit_cast<std::byte*>(
        bases[util::kIndexOfV<T, typename A::Records>] + (row * sizeof(T))
    );
  }

  template <typename A, typename T>
  static std::size_t RowOffset(std::size_t row, const RowTable<A>& segments) {
    constexpr std::size_t kCount =
        detail::RecordList<typename A::Records>::kCount;
    constexpr std::size_t kSegment = util::kIndexOfV<T, typename A::Records>;
    return segments[kCount + kSegment] +
           ((row - segments[kSegment]) * sizeof(T));
  }

  template <typename A>
  static std::size_t BlockBytes(
      std::size_t /*padded_instances*/, const RowTable<A>& segments
  ) {
    return segments[2 * detail::RecordList<typename A::Records>::kCount];
  }

 private:
  static constexpr std::size_t RoundUp(std::size_t value, std::size_t to) {
    return (value + to - 1) / to * to;
  }
};

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

}  // namespace achilles::engine::topology
