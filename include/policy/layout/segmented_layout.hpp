// A field whose rows sit in segments, each holding its own record type.

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

#include "policy/layout/planar_layout.hpp"
#include "util/simd_ops.hpp"
#include "util/tmp.hpp"

namespace achilles::policy::layout {

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
// LeafAssembler over several records (engine/assembly/assembler.hpp). Padding
// rows sit in no segment and take no space.
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

}  // namespace achilles::policy::layout
