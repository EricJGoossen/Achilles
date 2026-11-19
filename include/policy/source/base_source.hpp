// The base every source derives from, and SourceLike.

#pragma once

#include <concepts>
#include <cstddef>
#include <type_traits>

namespace achilles::policy::source {

// What a traversal asks a source: whether it can run batched, Of(rows,
// group) for each group's source index, and InitRow(rows) for where an op's
// Initialize runs.
template <typename S>
concept SourceLike = std::is_empty_v<S> && requires {
  { S::kBatchable } -> std::convertible_to<bool>;
};

// What every source derives from: Initialize at row 0, the base row, unless
// the source says otherwise. Never used polymorphically.
struct BaseSource {
  template <size_t Stride, typename Rows>
  static constexpr size_t InitRow(const Rows& /*rows*/) {
    return 0;
  }
};

}  // namespace achilles::policy::source
