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
  // -- e.g. engine::topology::OrderingPlan's own lane_size, which follows
  // whatever MathematicalT the hosted algorithms actually use (see
  // SimAllocator::RequiredLaneSize()). Taken explicitly rather than
  // re-derived from a hardcoded batch type, since a batch's own lane count
  // depends on its element width (a xsimd::batch<double> has half the
  // lanes of a same-register xsimd::batch<float>) -- hardcoding one here
  // would silently validate padding built for a different lane count.
  //
  // Holds whatever per-row source index its ordering planned -- a parent
  // for a tree ordering, a link into another ordering for a list one -- so
  // it checks nothing on construction: only the planner knows which
  // invariants its rows promise (see IsBatchSafe).
  JointTopology(std::span<const size_t> parents, size_t lane_size)
      : parents_(parents), lane_size_(lane_size) {}

  size_t Size() const { return parents_.size(); }

  size_t operator[](size_t i) const {
    assert(i < parents_.size());
    return parents_[i];
  }

  // True when every row's parent either comes strictly earlier or is out
  // of range (the reserved base row) -- what a single forward
  // TreeTraversal pass (engine/pass/traversals.hpp) needs to see a
  // joint's parent fully computed before the joint itself.
  // TopologicalOrdering guarantees this by construction; LinearOrdering
  // only when its archetypes have no real intra-archetype dependencies
  // (see LinearOrdering, orderings.hpp).
  bool IsForwardSafe() const {
    size_t size = Size();
    for (size_t row = 0; row < size; ++row) {
      size_t parent = parents_[row];
      if (parent >= row && parent < size) {
        return false;
      }
    }
    return true;
  }

  // True when no row and its source land in the same lane group -- what a
  // batched TreeTraversal needs so a lane group never reads a parent it is
  // itself still writing. Only meaningful when the sources are parents in
  // this same row space, so it's the tree orderings' planner that asserts
  // it (Ordering::BuildFromLocalOrders), not every topology.
  bool IsBatchSafe() const {
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

 private:
  std::span<const size_t> parents_;
  size_t lane_size_;
};
static_assert(TopologyLike<JointTopology>);

// A JointTopology whose rows are split into consecutive runs: segment k is
// rows [segments[k], segments[k + 1]). Only a list ordering with several
// lists has one (each list's rows one segment -- see
// engine::topology::ListOrdering); every other ordering's topology is a
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

// A JointTopology whose rows are pairs of another ordering's rows: each
// row's entry (operator[]) is its *source*, one row of the other ordering,
// and Partner(row) is the row it's paired with. Only a pair ordering has one
// (engine::topology::PairOrdering): a lane group of its rows shares one
// source, and lane t's partner is row t of one lane group of the other
// ordering -- or kNoPartner, for a padding lane (a pair that isn't one).
class PairTopology : public JointTopology {
 public:
  static constexpr size_t kNoPartner = static_cast<size_t>(-1);

  PairTopology(JointTopology rows, std::span<const size_t> partners)
      : JointTopology(rows), partners_(partners) {}

  size_t Partner(size_t row) const {
    assert(row < partners_.size());
    return partners_[row];
  }

 private:
  std::span<const size_t> partners_;
};
static_assert(TopologyLike<PairTopology>);

}  // namespace achilles::domain
