#pragma once

#include <bit>
#include <cassert>
#include <cstddef>
#include <limits>
#include <span>
#include <utility>
#include <vector>

#include "domain/archetype.hpp"
#include "domain/joint_topology.hpp"
#include "engine/memory/arena.hpp"

namespace achilles::engine::topology {

using domain::ArchetypeJointHandle;
using domain::ArchetypeTreeStructure;

// A copy of ArchetypeTreeStructure::kNoParent (domain/archetype.hpp),
// named for this header's own use: a padding row's sorted_to_physical_
// sentinel, not an archetype's own local-root marker.
inline constexpr std::size_t kNoPhysicalInstance =
    ArchetypeTreeStructure::kNoParent;

class Layout {
 public:
  // `parents` must hold exactly one more entry than `sorted_to_physical`
  // (PaddedSize() real/padding rows plus the one reserved base row) --
  // that's the shape Ordering::BuildFromLocalOrders always produces, and
  // everything below (BaseRowIndex, AllocateTopology, ...) relies on it.
  Layout(
      std::vector<std::vector<std::size_t>> physical_to_sorted,
      std::vector<ArchetypeJointHandle> sorted_to_physical,
      std::vector<std::size_t> parents,
      std::size_t lane_size
  )
      : physical_to_sorted_(std::move(physical_to_sorted)),
        sorted_to_physical_(std::move(sorted_to_physical)),
        parents_(std::move(parents)),
        lane_size_(lane_size) {
    assert(lane_size_ > 0 && "Layout: lane_size must be positive.");
    assert(
        parents_.size() == sorted_to_physical_.size() + 1 &&
        "Layout: parents must hold exactly one more row than "
        "sorted_to_physical (the reserved base row)."
    );
  }

  // Physical <-> sorted, for I/O keyed by real joint identity. Ops never
  // touch these -- a pass sees sorted rows only, via Topology().
  std::size_t ToSorted(std::size_t archetype_idx, std::size_t joint_idx) const {
    return physical_to_sorted_[archetype_idx][joint_idx];
  }
  ArchetypeJointHandle ToPhysical(std::size_t sorted) const {
    return sorted_to_physical_[sorted];
  }

  // Rows across every archetype's block -- real joints plus the padding
  // rows each dependency level (or archetype, for a non-topological
  // ordering) was rounded up to a lane multiple with. Excludes the one
  // reserved base row (see BaseRowIndex): sorted_to_physical_ holds
  // exactly one entry per padded row, real or not.
  std::size_t PaddedSize() const { return sorted_to_physical_.size(); }

  // True for a row no real (instance, joint) was ever assigned to --
  // Consolidate leaves sorted_to_physical's default sentinel
  // (kNoParent, kNoParent) exactly there and never overwrites it, so this
  // is just "does this row still hold that sentinel".
  bool IsPadding(std::size_t sorted) const {
    return sorted_to_physical_[sorted].instance_index == kNoPhysicalInstance;
  }

  // The one row every root archetype instance's own root joint (and every
  // padding row, per Ordering::BuildFromLocalOrders) resolves its external
  // parent to. Placed immediately after the last padded row: it's the
  // PaddedSize()'th row -- one past every row a Topology index ever names
  // as a traversal *target*, though a *parent* value may legitimately
  // equal it.
  std::size_t BaseRowIndex() const { return PaddedSize(); }

  // True when every real (non-padding) row's resolved parent either comes
  // strictly earlier in forward traversal order or is the reserved base
  // row -- exactly what a single forward TreeTraversal pass needs (see
  // engine/pass/traversals.hpp) to see a joint's parent already fully
  // computed by the time it processes that joint itself.
  // TopologicalOrdering's depth-sorted placement guarantees this by
  // construction; LinearOrdering's does too, but only because it's meant
  // for archetypes with no real intra-archetype dependencies to begin with
  // (see LinearOrdering's own doc comment, ordering_policy.hpp) -- this is
  // how SimAllocator confirms that assumption actually held for a given
  // set of archetypes, rather than trusting it blindly (see
  // detail::PickWidestLayout, sim_allocator.hpp: every ordering policy
  // produces the exact same PaddedSize() for one archetype set/lane_size,
  // so row count alone can't tell a safe Layout from an unsafe one -- this
  // can).
  bool IsForwardSafe() const {
    std::size_t size = PaddedSize();
    for (std::size_t row = 0; row < size; ++row) {
      std::size_t parent = parents_[row];
      if (parent >= row && parent < size) {
        return false;
      }
    }
    return true;
  }

  // How many rows every hosted field's block must actually be allocated
  // for: every padded row plus the one reserved base row, rounded up to
  // `lane_size` again so a batched load spanning the tail past the base
  // row never truncate-divides away a partial batch (see PlanarLayout's
  // own IsLaneMultiple contract). Any row beyond BaseRowIndex() here is
  // pure alignment filler -- never addressed by any real sorted index, any
  // padding index, or the base row itself -- so it only ever needs
  // zero/seed-safe content, never real data.
  std::size_t ViewInstanceCount() const {
    std::size_t needed = BaseRowIndex() + 1;
    return (needed + lane_size_ - 1) / lane_size_ * lane_size_;
  }

  std::size_t LaneSize() const { return lane_size_; }

  // Bytes AllocateTopology will carve out of whatever Arena it's given --
  // queryable up front so a caller can size that Arena's budget to include
  // this on top of every hosted field's own blocks.
  std::size_t TopologyBytes() const {
    return PaddedSize() * sizeof(std::size_t);
  }

  domain::JointTopology AllocateTopology(memory::Arena& arena) {
    // Only the padded rows (real + padding) are ever addressed as a
    // traversal *target* -- domain::JointTopology::Size() is exactly this
    // span's length, so a topology built any wider would hand
    // TreeTraversal the reserved base row (parents_[PaddedSize()], which
    // is self-referential) as an ordinary extra target to process. The
    // base row's own resolved value is still reachable: every real row's
    // *parent* value may legitimately equal BaseRowIndex(), it's just
    // never itself a row Apply visits.
    std::size_t size = PaddedSize();
    auto* storage = std::bit_cast<std::size_t*>(
        arena.Allocate(size * sizeof(std::size_t), alignof(std::size_t))
    );
    for (std::size_t i = 0; i < size; ++i) {
      storage[i] = parents_[i];
    }
    return domain::JointTopology(std::span<std::size_t>(storage, size));
  }

 private:
  std::vector<std::vector<std::size_t>> physical_to_sorted_;
  std::vector<ArchetypeJointHandle> sorted_to_physical_;
  std::vector<std::size_t> parents_;
  std::size_t lane_size_;
};

}  // namespace achilles::engine::topology
