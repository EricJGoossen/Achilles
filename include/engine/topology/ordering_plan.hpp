#pragma once

#include <algorithm>
#include <bit>
#include <cstddef>
#include <span>
#include <utility>
#include <vector>

#include "domain/archetype.hpp"
#include "domain/joint_layout.hpp"
#include "domain/joint_topology.hpp"
#include "engine/memory/arena.hpp"

namespace achilles::engine::topology {

using domain::ArchetypeJointHandle;

// What an ordering policy's Build produces: the row placement it chose,
// as plain arrays, plus the lane width it padded to. Scratch -- it lives
// only while SimAllocator builds the sim. Allocate copies it into the
// sim's Arena as the objects that persist: a JointTopology (each row's
// parent), a JointLayout (each row's joint) and, for a pair ordering, each
// row's partner. Topology()/Layout() give
// those same two views over this plan's own arrays, for use before (or
// without) allocating.
class OrderingPlan {
 public:
  // `parents`: per padded row, the row of its parent, or the reserved base
  // row (parents.size()) for a root instance's root joint and for padding.
  // `instance_offsets`, `sorted_rows`, `joints`: see domain::JointLayout's
  // constructor. A list ordering (ListOrdering) fills `parents` with link
  // rows into another ordering instead.
  OrderingPlan(
      std::vector<std::size_t> parents,
      std::vector<std::size_t> instance_offsets,
      std::vector<std::size_t> sorted_rows,
      std::vector<ArchetypeJointHandle> joints,
      std::size_t lane_size
  )
      : parents_(std::move(parents)),
        instance_offsets_(std::move(instance_offsets)),
        sorted_rows_(std::move(sorted_rows)),
        joints_(std::move(joints)),
        lane_size_(lane_size) {}

  // A pair ordering's plan (PairOrdering): the same, `parents` holding each
  // row's source, plus each row's partner (see domain::PairTopology).
  OrderingPlan(
      std::vector<std::size_t> parents,
      std::vector<std::size_t> partners,
      std::vector<std::size_t> instance_offsets,
      std::vector<std::size_t> sorted_rows,
      std::vector<ArchetypeJointHandle> joints,
      std::size_t lane_size
  )
      : parents_(std::move(parents)),
        instance_offsets_(std::move(instance_offsets)),
        sorted_rows_(std::move(sorted_rows)),
        joints_(std::move(joints)),
        partners_(std::move(partners)),
        lane_size_(lane_size) {}

  domain::JointTopology Topology() const { return {parents_, lane_size_}; }
  domain::JointLayout Layout() const {
    return {instance_offsets_, sorted_rows_, joints_};
  }

  // How many rows every field block must hold: every padded row plus the
  // reserved base row, rounded up to the lane width again so a batched
  // load spanning the tail never truncate-divides away a partial batch
  // (see PlanarLayout::IsLaneMultiple). Rows past the base row are filler.
  std::size_t ViewInstanceCount() const {
    std::size_t needed = parents_.size() + 1;
    return (needed + lane_size_ - 1) / lane_size_ * lane_size_;
  }

  // Arena bytes Allocate needs, including worst-case alignment slack for
  // each of its arrays.
  std::size_t AllocatedBytes() const {
    return ArrayBytes(parents_) + ArrayBytes(instance_offsets_) +
           ArrayBytes(sorted_rows_) + ArrayBytes(joints_) +
           ArrayBytes(partners_);
  }

  struct Allocated {
    domain::JointTopology topology;
    domain::JointLayout layout;
    std::span<const std::size_t> partners;
  };

  Allocated Allocate(memory::Arena& arena) const {
    return {
        domain::JointTopology(CopyInto(arena, parents_), lane_size_),
        domain::JointLayout(
            CopyInto(arena, instance_offsets_),
            CopyInto(arena, sorted_rows_),
            CopyInto(arena, joints_)
        ),
        CopyInto(arena, partners_)
    };
  }

 private:
  // Nothing for an empty array: CopyInto doesn't allocate one.
  template <typename T>
  static std::size_t ArrayBytes(const std::vector<T>& values) {
    return values.empty() ? 0 : (values.size() * sizeof(T)) + (alignof(T) - 1);
  }

  // An empty array (a list ordering with no entries) takes no Arena space.
  template <typename T>
  static std::span<const T> CopyInto(
      memory::Arena& arena, const std::vector<T>& values
  ) {
    if (values.empty()) {
      return {};
    }
    auto* storage =
        std::bit_cast<T*>(arena.Allocate(values.size() * sizeof(T), alignof(T))
        );
    std::ranges::copy(values, storage);
    return {storage, values.size()};
  }

  std::vector<std::size_t> parents_;
  std::vector<std::size_t> instance_offsets_;
  std::vector<std::size_t> sorted_rows_;
  std::vector<ArchetypeJointHandle> joints_;
  std::vector<std::size_t> partners_;
  std::size_t lane_size_;
};

}  // namespace achilles::engine::topology
