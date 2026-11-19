// A topology whose rows are split into consecutive segments.

#pragma once

#include <cstddef>
#include <span>

#include "domain/topology/joint_topology.hpp"
#include "domain/topology/topology_contract.hpp"

namespace achilles::domain::topology {

// A JointTopology whose rows are split into consecutive runs: segment k is
// rows [segments[k], segments[k + 1]). Only a list ordering with several
// lists has one (each list's rows one segment -- see
// policy::ordering::ListOrdering); every other ordering's topology is a
// plain JointTopology and carries none of this.
class SegmentedTopology : public JointTopology {
 public:
  SegmentedTopology(JointTopology rows, std::span<const size_t> segments)
      : JointTopology(rows), segments_(segments) {}

  size_t SegmentCount() const {
    return segments_.empty() ? 0 : segments_.size() - 1;
  }
  size_t SegmentBegin(size_t k) const {
    assert(k < SegmentCount());
    return segments_[k];
  }
  size_t SegmentEnd(size_t k) const {
    assert(k < SegmentCount());
    return segments_[k + 1];
  }

 private:
  std::span<const size_t> segments_;
};
static_assert(TopologyLike<SegmentedTopology>);

}  // namespace achilles::domain::topology
