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
  uint8_t segment;
  uint8_t parent_joint;
};
class JointTopology {
 public:
  JointTopology(
      std::span<std::span<size_t>> data, std::span<SegmentData> segment_data
  )
      : data_(data), segment_data_(segment_data) {
    assert(CheckDataConsistency() && "Inconsistent data in JointTopology");
    assert(
        CheckBatchSafety() &&
        "A parent and child joint land in the same SIMD batch -- the "
        "topological sort must pad each dependency level to a multiple "
        "of lane_size so batching never crosses a parent/child edge."
    );
  };

  size_t Size() const {
    size_t total_size = 0;
    for (const SegmentData& sd : segment_data_) {
      total_size += data_[sd.segment].size();
    }
    return total_size;
  }

  size_t operator[](size_t i) const {
    assert(i < Size() && "Index out of range in Layout");
    for (const SegmentData& sd : segment_data_) {
      size_t seg_size = data_[sd.segment].size();
      if (i < seg_size) {
        size_t idx = data_[sd.segment][i];
        return idx == kInterSegmentParent ? sd.parent_joint : idx;
      }
      i -= seg_size;
    }
    throw std::out_of_range("Index out of range in Layout");
  }

 private:
  bool CheckDataConsistency() const {
    return std::ranges::all_of(segment_data_, [this](const SegmentData& sd) {
      return sd.segment < data_.size();
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

  std::span<std::span<size_t>> data_;
  std::span<SegmentData> segment_data_;
};
static_assert(TopologyLike<JointTopology>);

}  // namespace achilles::domain::topology
