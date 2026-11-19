// Each field's rows laid out one after another, a fixed stride apart.

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

#include "policy/layout/base_layout.hpp"
#include "util/simd_ops.hpp"
#include "util/tmp.hpp"

namespace achilles::policy::layout {

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
struct PlanarLayout : BaseLayout {
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
};

}  // namespace achilles::policy::layout
