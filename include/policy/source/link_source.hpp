// Each row's source is its link into another ordering.

#pragma once

#include <cstddef>

#include "domain/topology/joint_topology.hpp"
#include "domain/topology/topology_contract.hpp"
#include "policy/source/base_source.hpp"

namespace achilles::policy::source {

// The row's *link* (ArgIndex::kLink) -- for a list ordering (topology::
// ListOrdering), whose topology holds, per row, the row in another
// ordering's row space that entry is attached to (e.g. the joint a sensor
// is mounted on). An Op reads that other ordering's fields at kLink and its
// own list-ordered ones at kTarget. Not batchable: links point anywhere, so
// a lane group of entries has no single source group to load. Initialize
// runs at row 0 of the list itself.
struct LinkSource : BaseSource {
  static constexpr bool kBatchable = false;

  template <size_t Stride, domain::topology::TopologyLike Topology>
  static constexpr size_t Of(const Topology& links, size_t row) {
    return links[row];
  }
};
static_assert(SourceLike<LinkSource>);

}  // namespace achilles::policy::source
