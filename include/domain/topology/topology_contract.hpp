// What a topology a traversal walks must provide.

#pragma once

#include <concepts>
#include <cstddef>

namespace achilles::domain::topology {

template <typename T>
concept TopologyLike = requires(const T& topology) {
  { topology.Size() } -> std::convertible_to<std::size_t>;
  { topology[0] } -> std::convertible_to<std::size_t>;
};

}  // namespace achilles::domain::topology
