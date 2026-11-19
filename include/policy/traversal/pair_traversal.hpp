// Every pair of a pair ordering, a lane group of partners per source row.

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
#include "policy/traversal/row_traversal.hpp"

namespace achilles::policy::traversal {

// Every lane group of a pair ordering (policy::ordering::PairOrdering), handed
// its source (ArgIndex::kSource, broadcast), its partners (ArgIndex::kPartner)
// and which lanes are pairs (ArgIndex::kStructure) -- the pairs a broad
// phase, narrow phase or contact solver walks. A Step over pairs is given
// the pair topology after the rows (the other passes' topology), since a
// pass over pairs usually shares a Step with passes over the bodies they
// pair; this walks that second argument as its rows. Its Op's Initialize
// runs even when there are no pairs: a pass over pairs that reduces into a
// whole-sim value (policy::indexing::GlobalIndexing) starts that value from it,
// and passes over the bodies may add to the same value after it.
template <Direction Dir, size_t Stride>
struct PairTraversal : BaseTraversal<Dir, Stride> {
  using Walk = RowTraversal<
      Dir,
      Stride,
      source::PairSource,
      range::AllRange,
      scan::PairScan>;

  template <typename Callable, typename Rows, typename... Rest>
  static constexpr void Apply(
      const Callable& callable,
      const Rows& /*rows*/,
      const domain::topology::PairTopology& pairs,
      const Rest&... /*rest*/
  ) {
    Walk::Apply(callable, pairs);
  }

  template <typename Callable, typename Rows, typename... Rest>
  static constexpr void InitOp(
      const Callable& callable,
      const Rows& /*rows*/,
      const domain::topology::PairTopology& pairs,
      const Rest&... /*rest*/
  ) {
    callable.Initialize(source::PairSource::InitRow<Stride>(pairs));
  }
};

using ForwardPairTraversal = PairTraversal<Direction::kForward, 1>;

using BackwardPairTraversal = PairTraversal<Direction::kBackward, 1>;

static_assert(
    TraversalLike<ForwardPairTraversal>,
    "ForwardPairTraversal must satisfy TraversalLike concept"
);

static_assert(
    TraversalLike<BackwardPairTraversal>,
    "BackwardPairTraversal must satisfy TraversalLike concept"
);

}  // namespace achilles::policy::traversal
