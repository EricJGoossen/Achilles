#pragma once

#include <cassert>
#include <cstddef>
#include <span>

#include "domain/archetype.hpp"

namespace achilles::domain {

// Where every joint lives: the twin of JointTopology (who every joint's
// parent is). Maps a real joint -- (global instance, local joint), the same
// ArchetypeJointHandle the archetype tree names attachments with -- to the
// sorted row an ordering policy placed it at, and back. Rows no real joint
// occupies are padding.
//
// A view over three arrays it doesn't own (engine::topology::OrderingPlan
// builds them; SimAllocator copies them into the sim's Arena), so it's
// trivially copyable and touches no heap.
class JointLayout {
 public:
  // Marks a padding row's entry in `joints`.
  static constexpr std::size_t kNoInstance = ArchetypeTreeStructure::kNoParent;

  // `instance_offsets[g]` is where global instance g's joints start in
  // `sorted_rows` (one entry per instance, plus a final end offset);
  // `sorted_rows` holds one sorted row per real joint; `joints` holds one
  // entry per padded row -- the joint at that row, or {kNoInstance,
  // kNoInstance} for padding.
  JointLayout(
      std::span<const std::size_t> instance_offsets,
      std::span<const std::size_t> sorted_rows,
      std::span<const ArchetypeJointHandle> joints
  )
      : instance_offsets_(instance_offsets),
        sorted_rows_(sorted_rows),
        joints_(joints) {
    assert(
        !instance_offsets_.empty() &&
        instance_offsets_.back() == sorted_rows_.size() &&
        "JointLayout: instance_offsets must end at sorted_rows.size()."
    );
  }

  // Inverses of each other: ToSorted(ToPhysical(row)) == row for every
  // real row.
  std::size_t ToSorted(ArchetypeJointHandle joint) const {
    assert(joint.instance_index + 1 < instance_offsets_.size());
    std::size_t first = instance_offsets_[joint.instance_index];
    assert(
        first + joint.joint_index < instance_offsets_[joint.instance_index + 1]
    );
    return sorted_rows_[first + joint.joint_index];
  }
  ArchetypeJointHandle ToPhysical(std::size_t row) const {
    return joints_[row];
  }

  bool IsPadding(std::size_t row) const {
    return joints_[row].instance_index == kNoInstance;
  }

  // Real plus padding rows. Excludes the reserved base row.
  std::size_t PaddedSize() const { return joints_.size(); }

  // The row every root instance's root joint (and every padding row)
  // names as its parent -- one past the last padded row.
  std::size_t BaseRowIndex() const { return PaddedSize(); }

  std::size_t InstanceCount() const { return instance_offsets_.size() - 1; }

  // Where each instance's joints begin among the sorted entries, then their
  // total: instance i's are [InstanceOffsets()[i], InstanceOffsets()[i + 1]).
  // A list ordering numbers list k as instance k with its rows in order, so
  // for it these are each list's rows -- its segments (see
  // domain::SegmentedTopology).
  std::span<const std::size_t> InstanceOffsets() const {
    return instance_offsets_;
  }

 private:
  std::span<const std::size_t> instance_offsets_;
  std::span<const std::size_t> sorted_rows_;
  std::span<const ArchetypeJointHandle> joints_;
};

}  // namespace achilles::domain
