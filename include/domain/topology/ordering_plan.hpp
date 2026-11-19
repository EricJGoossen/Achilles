// An ordering's row placement as plain arrays: what an ordering policy's Build
// produces, before the sim copies it into the arena.

#pragma once

#include <cstddef>
#include <utility>
#include <vector>

#include "domain/layout/joint_layout.hpp"
#include "domain/layout/joint_records.hpp"
#include "domain/topology/joint_topology.hpp"

namespace achilles::domain::topology {

// What an ordering policy's Build produces: the row placement it chose,
// as plain arrays, plus the lane width it padded to. Scratch -- it lives
// only while SimAllocator builds the sim, which copies it into the sim's
// Arena as the objects that persist (engine::allocation::PlanCopy): a
// JointTopology (each row's parent), a JointLayout (each row's joint) and, for
// a pair ordering, each row's partner. Topology()/Layout() give those same two
// views over this plan's own arrays, for use before (or without) allocating.
struct OrderingPlan {
  // `parents`: per padded row, the row of its parent, or the reserved base
  // row (parents.size()) for a root instance's root joint and for padding.
  // `instance_offsets`, `sorted_rows`, `joints`: see domain::JointLayout's
  // constructor. A list ordering (ListOrdering) fills `parents` with link
  // rows into another ordering instead.
  OrderingPlan(
      std::vector<std::size_t> parents,
      std::vector<std::size_t> instance_offsets,
      std::vector<std::size_t> sorted_rows,
      std::vector<domain::layout::JointRef> joints,
      std::size_t lane_size
  )
      : parents(std::move(parents)),
        instance_offsets(std::move(instance_offsets)),
        sorted_rows(std::move(sorted_rows)),
        joints(std::move(joints)),
        lane_size(lane_size) {}

  // A pair ordering's plan (PairOrdering): the same, `parents` holding each
  // row's source, plus each row's partner (see domain::PairTopology).
  OrderingPlan(
      std::vector<std::size_t> parents,
      std::vector<std::size_t> partners,
      std::vector<std::size_t> instance_offsets,
      std::vector<std::size_t> sorted_rows,
      std::vector<domain::layout::JointRef> joints,
      std::size_t lane_size
  )
      : parents(std::move(parents)),
        instance_offsets(std::move(instance_offsets)),
        sorted_rows(std::move(sorted_rows)),
        joints(std::move(joints)),
        partners(std::move(partners)),
        lane_size(lane_size) {}

  JointTopology Topology() const { return {parents, lane_size}; }
  domain::layout::JointLayout Layout() const {
    return {instance_offsets, sorted_rows, joints};
  }

  // How many rows every field block must hold: every padded row plus the
  // reserved base row, rounded up to the lane width again so a batched
  // load spanning the tail never truncate-divides away a partial batch
  // (see PlanarLayout::IsLaneMultiple). Rows past the base row are filler.
  std::size_t ViewInstanceCount() const {
    std::size_t needed = parents.size() + 1;
    return (needed + lane_size - 1) / lane_size * lane_size;
  }

  // unused, including worst-case alignment slack for
  // each of its arrays.
  std::size_t AllocatedBytes() const {
    return ArrayBytes(parents) + ArrayBytes(instance_offsets) +
           ArrayBytes(sorted_rows) + ArrayBytes(joints) + ArrayBytes(partners);
  }

  std::vector<std::size_t> parents;
  std::vector<std::size_t> instance_offsets;
  std::vector<std::size_t> sorted_rows;
  std::vector<domain::layout::JointRef> joints;
  std::vector<std::size_t> partners;
  std::size_t lane_size;
};

}  // namespace achilles::domain::topology
