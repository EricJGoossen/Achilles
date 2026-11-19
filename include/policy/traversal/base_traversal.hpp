// The base every traversal derives from, its concept, and the direction
// it walks in.
// Doc: docs/code/policy_traversal_base_traversal.md

#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <type_traits>
#include <utility>

#include "domain/topology/joint_topology.hpp"
#include "domain/topology/pair_topology.hpp"

namespace achilles::policy::traversal {

enum class Direction : uint8_t { kForward, kBackward };

// What every traversal derives from: its direction and stride. Never used
// polymorphically: nothing holds or calls one through it.
template <Direction Dir, size_t Stride>
struct BaseTraversal {
  static constexpr Direction kDirection = Dir;
  static constexpr size_t kStride = Stride;
};

namespace detail {

// Satisfies both shapes a traversal's second argument can take: a
// Topology-like object (Size() + operator[]) for TreeTraversal, and a bare
// size_t (via the implicit conversion) for LinearTraversal -- so a single
// requires-block in TraversalLike below can probe either kind of traversal
// without knowing up front which one T is.
struct TraversalProbe {
  size_t Size() const { return 0; }
  size_t operator[](size_t) const { return 0; }
  size_t SegmentBegin(size_t) const { return 0; }
  size_t SegmentEnd(size_t) const { return 0; }
  explicit operator size_t() const { return 0; }
};

}  // namespace detail

// A traversal op's counterpart to OpLike: default-constructible, exposing
// the same `kDirection` metadata every traversal op carries, and providing
// static `Apply`/`ApplyToBase` entry points shaped like the traversals
// below -- Apply drives a callable over every (target, source) index pair
// (forward or backward; source is a parent group, the target itself, or a
// policy::indexing::BroadcastIndex row, depending on the traversal),
// ApplyToBase calls the callable once, with just the single base-row index, for
// whichever Op::Initialize a pass's OpInvoker exposes (see OpHasInit/OpInvoker
// in engine/spec/op_contract.hpp and engine/op_invoker.hpp -- not every Op
// has one, so RunPass in algorithm_step.hpp only calls ApplyToBase when it
// does).
template <typename T>
concept TraversalLike =
    std::is_default_constructible_v<T> &&
    requires {
      { T::kDirection } -> std::same_as<const Direction&>;
    } &&
    ((
         requires(
             void (&callable)(size_t, size_t), detail::TraversalProbe topology
         ) {
           { T::Apply(callable, topology) } -> std::same_as<void>;
         } &&
         requires(void (&callable)(size_t), detail::TraversalProbe topology) {
           { T::InitOp(callable, topology) } -> std::same_as<void>;
         }
     ) ||
     // A pass over pairs (PairTraversal) takes the pair topology after the
     // rows.
     (
         requires(
             void (&callable)(size_t, size_t),
             detail::TraversalProbe topology,
             const domain::topology::PairTopology& pairs
         ) {
           { T::Apply(callable, topology, pairs) } -> std::same_as<void>;
         } &&
         requires(
             void (&callable)(size_t),
             detail::TraversalProbe topology,
             const domain::topology::PairTopology& pairs
         ) {
           { T::InitOp(callable, topology, pairs) } -> std::same_as<void>;
         }
     ));

// A traversal, shaped like an Op: a stateless callable, default-constructed
// fresh per use, invoked with the OpInvoker plus *everything* Step (see
// engine/algorithm_step.hpp) was called with after `view` -- not just the
// one argument this particular traversal cares about -- so one Step call
// can mix passes whose traversals want different things (a JointTopology, a
// bare count, ...) without every pass having to agree on a single shared
// argument list. `kDirection` mirrors an Op's own static metadata
// (kInputs/kOutputs): it's how TraversalLike above tells a real traversal
// apart from an arbitrary type, and it's queryable independent of Apply.
//
// Stride is how many consecutive raw rows one traversal step actually
// covers -- 1 (the default) for a scalar Op, or a batched Op's own T's lane
// count (util::LaneCountOf<T>()) otherwise. A JointTopology's own Size()/
// operator[] are always in raw-row units (one entry per real+padding row --
// see JointLayout::PaddedSize()), but OpInvoker feeds whatever
// index this traversal hands it straight into View::Load/Store<T>, which
// for a batched T addresses storage in units of Stride-sized *groups*, not
// one raw row at a time (see View's own comment on Value/Load: "instance
// index directly when T is scalar, batch index when T is batched"). So a
// traversal driving a batched Op must walk Size()/Stride groups, not
// Size() raw rows, and must divide each raw-row parent value by Stride too
// -- both rows of a Stride-sized batch group always resolve to the same
// parent GROUP, since SimAllocator always pads every level's own per-
// instance block out to a multiple of the widest Stride any hosted field
// needs (see policy::ordering::OrderingPlan::ViewInstanceCount()), so a group
// never straddles two different tree positions. Stride=1 (every existing scalar
// caller) makes every division here a no-op, so this is a strictly
// backward-compatible extension.

}  // namespace achilles::policy::traversal
