// Each row group's source is its parent's group.

#pragma once

#include <cstddef>

#include "domain/topology/joint_topology.hpp"
#include "domain/topology/topology_contract.hpp"
#include "policy/source/base_source.hpp"

namespace achilles::policy::source {

// The row's parent group, in the same row space (ArgIndex::kParent) -- a
// tree ordering's topology. Batchable because the tree planner guarantees
// no parent shares a lane group with its child (see JointTopology::
// IsBatchSafe). Initialize runs at row 0's parent: the base row a root
// joint reads as its parent, seeded before the walk.
struct ParentSource : BaseSource {
  static constexpr bool kBatchable = true;

  template <size_t Stride, domain::topology::TopologyLike Topology>
  static constexpr size_t Of(const Topology& parents, size_t group) {
    return parents[group * Stride] / Stride;
  }
  template <size_t Stride, domain::topology::TopologyLike Topology>
  static constexpr size_t InitRow(const Topology& parents) {
    return parents[0] / Stride;
  }
};
static_assert(SourceLike<ParentSource>);

}  // namespace achilles::policy::source
