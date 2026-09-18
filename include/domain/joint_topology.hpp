#pragma once

#include <cassert>
#include <cstddef>
#include <span>
#include <xsimd/xsimd.hpp>

namespace achilles::domain {

template <typename T>
concept TopologyLike = requires(const T& topology) {
  { topology.Size() } -> std::convertible_to<std::size_t>;
  { topology[0] } -> std::convertible_to<std::size_t>;
};

class JointTopology {
 public:
  explicit JointTopology(std::span<size_t> parents) : parents_(parents) {
    assert(
        CheckBatchSafety() &&
        "A parent and child joint land in the same SIMD batch -- the "
        "topological sort must pad each dependency level to a multiple "
        "of lane_size so batching never crosses a parent/child edge."
    );
  };

  size_t Size() const { return parents_.size(); }

  size_t operator[](size_t i) const {
    assert(i < parents_.size());
    return parents_[i];
  }

 private:
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

  std::span<size_t> parents_;
};
static_assert(TopologyLike<JointTopology>);

}  // namespace achilles::domain
