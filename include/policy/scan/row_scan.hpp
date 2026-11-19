// Each row group against every real row of a second layout.

#pragma once

#include <cstddef>
#include <cstdint>

#include "policy/scan/base_scan.hpp"

namespace achilles::policy::scan {

struct RowScan : BaseScan {
  static constexpr bool kBatchable = false;

  template <
      size_t Stride,
      typename Callable,
      typename Rows,
      typename Scene,
      typename... Rest>
  static constexpr void Visit(
      const Callable& callable,
      const Rows& /*rows*/,
      size_t group,
      size_t source,
      const Scene& scene,
      const Rest&... /*rest*/
  ) {
    const size_t rows = scene.PaddedSize();
    for (size_t row = 0; row < rows; ++row) {
      if (!scene.IsPadding(row)) {
        callable(group, source, row);
      }
    }
  }
  template <size_t Stride, typename Callable, typename Rows>
  static constexpr void Visit(
      const Callable& /*callable*/,
      const Rows& /*rows*/,
      size_t /*group*/,
      size_t /*source*/
  ) {}

  template <typename Scene, typename... Rest>
  static constexpr bool HasWork(const Scene& scene, const Rest&... /*rest*/) {
    return scene.PaddedSize() > 0;
  }
  static constexpr bool HasWork() { return false; }
};
static_assert(ScanLike<RowScan>);

}  // namespace achilles::policy::scan
