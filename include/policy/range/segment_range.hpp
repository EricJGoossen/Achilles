// Only segment K's rows.

#pragma once

#include <cstddef>

#include "policy/range/base_range.hpp"

namespace achilles::policy::range {

template <size_t K>
struct SegmentRange {
  template <size_t Stride, typename Rows>
    requires(Stride == 1)
  static constexpr size_t Begin(const Rows& rows) {
    return rows.SegmentBegin(K);
  }
  template <size_t Stride, typename Rows>
    requires(Stride == 1)
  static constexpr size_t End(const Rows& rows) {
    return rows.SegmentEnd(K);
  }
};
static_assert(RangeLike<SegmentRange<0>>);

}  // namespace achilles::policy::range
