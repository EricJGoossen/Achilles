// One element for the whole sim.

#pragma once

#include <concepts>
#include <cstddef>
#include <tuple>
#include <type_traits>

#include "domain/layout/joint_layout.hpp"
#include "policy/indexing/base_indexing.hpp"

namespace achilles::policy::indexing {

// One element for the whole field, whatever the row count -- a quantity the
// whole sim shares, e.g. a dot product summed over every row, or a step
// length an iterative solver picks from it. Every index resolves to that
// one element, by load width:
//
//   - a scalar load (LoadLanes = 1) reads/writes it plainly at index 0 --
//     what an Op driven by a GlobalTraversal (policy/traversal/)
//     works on, once per pass, to update the value itself;
//   - a batched load resolves to BroadcastIndex{0}: as an input every lane
//     reads the one value, and as an output each call's lanes are summed
//     into it (see BroadcastIndex above) -- how a tree, linear or pair pass
//     reduces over its rows into it.
//
// Nothing here is ever padding: the one element is always real.
struct GlobalIndexing : BaseIndexing {
  static constexpr std::size_t Count(std::size_t /*rows*/) { return 1; }

  template <std::size_t Slot, std::size_t LoadLanes>
    requires(LoadLanes == 1)
  static constexpr std::size_t
  Resolve(const auto& /*indices*/, std::size_t /*rows*/) {
    return 0;
  }

  template <std::size_t Slot, std::size_t LoadLanes>
    requires(LoadLanes > 1)
  static constexpr BroadcastIndex
  Resolve(const auto& /*indices*/, std::size_t /*rows*/) {
    return BroadcastIndex{0};
  }
};

static_assert(IndexingPolicyLike<GlobalIndexing>);

}  // namespace achilles::policy::indexing
