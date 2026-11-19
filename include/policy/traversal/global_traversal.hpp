// One call for the whole sim, at its single global row.

#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <type_traits>
#include <utility>

#include "domain/topology/joint_topology.hpp"
#include "domain/topology/pair_topology.hpp"
#include "policy/traversal/base_traversal.hpp"

namespace achilles::policy::traversal {

// One call, at index 0 -- for an Op whose work is a single value the whole
// sim shares rather than anything per row: a scalar Op over
// policy::indexing::GlobalIndexing fields, e.g. an iterative solver turning the
// sums a reduction pass just left in them into its next step length. Takes
// (and ignores) whatever the Step call's other traversals take, so it sits
// in the same Ops list as the passes that reduce into those fields and the
// ones that read the result. Always Stride 1: there's only the one element
// to read. Initialize, if the Op has one, runs at that same index first.
struct GlobalTraversal : BaseTraversal<Direction::kForward, 1> {
  template <typename Callable, typename... Rest>
  static constexpr void Apply(const Callable& callable, const Rest&...) {
    callable(0, 0);
  }

  template <typename Callable, typename... Rest>
  static constexpr void InitOp(const Callable& callable, const Rest&...) {
    callable.Initialize(0);
  }
};

static_assert(
    TraversalLike<GlobalTraversal>,
    "GlobalTraversal must satisfy TraversalLike concept"
);

}  // namespace achilles::policy::traversal
