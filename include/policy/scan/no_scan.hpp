// Each row group on its own, paired against nothing.

#pragma once

#include <cstddef>
#include <cstdint>

#include "policy/scan/base_scan.hpp"

namespace achilles::policy::scan {

// What a RowTraversal does at each row group it visits: call once with
// (group, source) -- NoScan -- or, RowScan, pair the group with every real
// row of the JointLayout a Step was given second (its padding rows, which
// hold no real entry, skipped), calling once per pair with that row as a
// third index (ArgIndex::kScene). Scanning is for an Op relating each entry
// of one list to every row of another -- a lidar against every body's
// shape, say -- that still reads the entry's own source (its link). Given
// no second argument, a scan has nothing to pair against and does nothing.
struct NoScan : BaseScan {
  static constexpr bool kBatchable = true;

  template <
      size_t Stride,
      typename Callable,
      typename Rows,
      typename Source,
      typename... Rest>
  static constexpr void Visit(
      const Callable& callable,
      const Rows& /*rows*/,
      size_t group,
      const Source& source,
      const Rest&... /*rest*/
  ) {
    callable(group, source);
  }
};
static_assert(ScanLike<NoScan>);

}  // namespace achilles::policy::scan
