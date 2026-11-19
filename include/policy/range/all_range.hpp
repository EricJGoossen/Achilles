// Every row group of the rows walked.

#pragma once

#include <cstddef>

#include "policy/range/base_range.hpp"

namespace achilles::policy::range {

namespace detail {

// How many raw rows a RowTraversal walks: a bare count as-is, or a
// topology's Size().
constexpr size_t RowCount(size_t rows) { return rows; }

template <typename Rows>
  requires requires(const Rows& rows) { rows.Size(); }

constexpr size_t RowCount(const Rows& rows) {
  return rows.Size();
}

}  // namespace detail

// Which row groups a RowTraversal walks, [Begin, End) in groups of Stride:
// every one (AllRange), or only segment K of a topology split into segments
// (one per list of a policy::ordering::ListOrdering -- see
// domain::JointTopology::SegmentBegin), one row at a time since a segment
// needn't start on a lane boundary.
struct AllRange {
  template <size_t Stride, typename Rows>
  static constexpr size_t Begin(const Rows& /*rows*/) {
    return 0;
  }
  template <size_t Stride, typename Rows>
  static constexpr size_t End(const Rows& rows) {
    return detail::RowCount(rows) / Stride;
  }
};
static_assert(RangeLike<AllRange>);

}  // namespace achilles::policy::range
