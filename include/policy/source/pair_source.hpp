// Each pair group's source row, broadcast to every lane.

#pragma once

#include <cstddef>

#include "domain/topology/joint_topology.hpp"
#include "domain/topology/pair_topology.hpp"
#include "domain/topology/topology_contract.hpp"
#include "policy/indexing/base_indexing.hpp"
#include "policy/source/base_source.hpp"

namespace achilles::policy::source {

// The row group's *source* (ArgIndex::kSource) -- for a pair ordering
// (policy::ordering::PairOrdering), whose lane groups each pair one source row
// of another ordering with a lane group of that ordering's rows (see
// domain::PairTopology). Handed as a policy::indexing::BroadcastIndex: an Op
// reads the source's fields once, splatted across every lane, and an output
// written there sums the group's lanes into it. Batchable, since a lane
// group shares its source. Initialize runs at row 0 of the pairs.
struct PairSource : BaseSource {
  static constexpr bool kBatchable = true;

  template <size_t Stride>
  static constexpr policy::indexing::BroadcastIndex Of(
      const domain::topology::PairTopology& pairs, size_t group
  ) {
    return {pairs[group * Stride]};
  }
};
static_assert(SourceLike<PairSource>);

}  // namespace achilles::policy::source
