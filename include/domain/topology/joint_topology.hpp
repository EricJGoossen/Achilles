#pragma once

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <limits>
#include <span>
#include <stdexcept>
#include <xsimd/xsimd.hpp>

#include "topology_contract.hpp"

namespace achilles::domain::topology {

static constexpr size_t kInterSegmentParent = std::numeric_limits<size_t>::max(
);  // TODO: temp, should be defined in the topological sort that builds the
    // Layout, not here.

struct SegmentData {
  uint16_t segment;
  uint16_t parent_joint;
};
class JointTopology {
 public:
  JointTopology(
      std::span<std::span<size_t>> segment_layouts,
      std::span<SegmentData> segment_data
  )
      : segment_layouts_(segment_layouts), segment_data_(segment_data) {
    assert(CheckDataConsistency() && "Inconsistent data in JointTopology");
    assert(
        CheckBatchSafety() &&
        "A parent and child joint land in the same SIMD batch -- the "
        "topological sort must pad each dependency level to a multiple "
        "of lane_size so batching never crosses a parent/child edge."
    );
    assert(
        segment_data_.size() <= std::numeric_limits<uint16_t>::max() &&
        "Too many segments for JointTopology"
    );
  };

  size_t Size() const {
    size_t total_size = 0;
    for (const SegmentData& sd : segment_data_) {
      total_size += segment_layouts_[sd.segment].size();
    }
    return total_size;
  }

  size_t operator[](size_t i) const {
    assert(i < Size() && "Index out of range in Layout");
    size_t base = 0;
    for (const SegmentData& sd : segment_data_) {
      size_t idx = i - base;
      size_t seg_size = segment_layouts_[sd.segment].size();
      if (idx < seg_size) {
        // The stored value is already a global/flattened index (or the
        // kInterSegmentParent sentinel) -- only the lookup into this
        // segment's own array is segment-relative, via idx.
        size_t stored = segment_layouts_[sd.segment][idx];
        return stored == kInterSegmentParent ? sd.parent_joint : stored;
      }
      base += seg_size;
    }
    throw std::out_of_range("Index out of range in Layout");
  }

 private:
  bool CheckDataConsistency() const {
    return std::ranges::all_of(segment_data_, [this](const SegmentData& sd) {
      return sd.segment < segment_layouts_.size();
    });
  }

  bool CheckBatchSafety() const {
    size_t lane_size = xsimd::batch<float>::size;

    if (lane_size <= 1) {
      return true;
    }
    size_t size = Size();
    for (size_t i = 0; i < size; ++i) {
      size_t parent = (*this)[i];
      if (parent >= size) {
        continue;
      }
      if (parent / lane_size == i / lane_size) {
        return false;
      }
    }
    return true;
  }

  std::span<std::span<size_t>> segment_layouts_;
  std::span<SegmentData> segment_data_;
};
static_assert(TopologyLike<JointTopology>);

}  // namespace achilles::domain::topology
