// The base every scan derives from, and ScanLike.

#pragma once

#include <concepts>
#include <type_traits>

namespace achilles::policy::scan {

// What a traversal asks a scan: whether it can run batched, Visit for each
// row group, and HasWork for whether an op's Initialize should run at all.
template <typename S>
concept ScanLike = std::is_empty_v<S> && requires {
  { S::kBatchable } -> std::convertible_to<bool>;
  { S::HasWork() } -> std::convertible_to<bool>;
};

// What every scan derives from: always work to do, unless the scan says
// otherwise. Never used polymorphically.
struct BaseScan {
  template <typename... Rest>
  static constexpr bool HasWork(const Rest&... /*rest*/) {
    return true;
  }
};

}  // namespace achilles::policy::scan
