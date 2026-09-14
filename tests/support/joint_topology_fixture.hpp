#pragma once

#include <cstddef>
#include <span>
#include <utility>
#include <vector>

#include "domain/topology/joint_topology.hpp"

namespace achilles::test_support {

// JointTopology (domain/topology/joint_topology.hpp) only stores spans --
// it never owns the backing arrays -- so a test needs somewhere for the
// real storage to live for at least as long as the JointTopology built
// over it. Bundles that storage with the JointTopology itself so a test
// gets one movable object. Used by both domain_topology_joint_topology.cpp
// (testing JointTopology itself) and algorithms_aba_aba_step.cpp (which
// needs a real ABATopology = JointTopology to drive aba::Step).
struct TopologyFixture {
  std::vector<std::vector<std::size_t>> segments;
  std::vector<std::span<std::size_t>> segment_spans;
  std::vector<domain::topology::SegmentData> segment_data;
  domain::topology::JointTopology topology;

  TopologyFixture(
      std::vector<std::vector<std::size_t>> segments_in,
      std::vector<domain::topology::SegmentData> segment_data_in
  )
      : segments(std::move(segments_in)),
        segment_data(std::move(segment_data_in)),
        topology(BuildSpans(), std::span<domain::topology::SegmentData>(segment_data)) {}

  std::span<std::span<std::size_t>> BuildSpans() {
    segment_spans.clear();
    for (auto& seg : segments) {
      segment_spans.emplace_back(seg);
    }
    return {segment_spans};
  }
};

}  // namespace achilles::test_support
