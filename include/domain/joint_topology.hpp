#pragma once

#include <cassert>
#include <cstddef>
#include <span>

namespace achilles::domain {

template <typename T>
concept TopologyLike = requires(const T& topology) {
  { topology.Size() } -> std::convertible_to<std::size_t>;
  { topology[0] } -> std::convertible_to<std::size_t>;
};

class JointTopology {
 public:
  // `lane_size` is the real SIMD lane width the caller padded `parents` to
  // -- e.g. engine::topology::Layout's own lane_size_, which follows
  // whatever MathematicalT the hosted algorithms actually use (see
  // SimAllocator::RequiredLaneSize()). Taken explicitly rather than
  // re-derived from a hardcoded batch type, since a batch's own lane count
  // depends on its element width (a xsimd::batch<double> has half the
  // lanes of a same-register xsimd::batch<float>) -- hardcoding one here
  // would silently validate padding built for a different lane count.
  explicit JointTopology(std::span<size_t> parents, size_t lane_size)
      : parents_(parents), lane_size_(lane_size) {
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
    size_t lane_size = lane_size_;

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
  size_t lane_size_;
};
static_assert(TopologyLike<JointTopology>);

}  // namespace achilles::domain
