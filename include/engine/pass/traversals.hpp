#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <type_traits>
#include <utility>

#include "domain/joint_topology.hpp"
#include "engine/pass/row_policies.hpp"
#include "engine/topology/indexing_policy.hpp"

namespace achilles::engine::pass {

enum class Direction : uint8_t { kForward, kBackward };

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
// topology::BroadcastIndex row, depending on the traversal), ApplyToBase
// calls the callable once, with just the
// single base-row index, for whichever Op::Initialize a pass's OpInvoker
// exposes (see OpHasInit/OpInvoker in engine/op_contract.hpp and
// engine/op_invoker.hpp -- not every Op has one, so RunPass in
// algorithm_step.hpp only calls ApplyToBase when it does).
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
             const domain::PairTopology& pairs
         ) {
           { T::Apply(callable, topology, pairs) } -> std::same_as<void>;
         } &&
         requires(
             void (&callable)(size_t),
             detail::TraversalProbe topology,
             const domain::PairTopology& pairs
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
// needs (see topology::OrderingPlan::ViewInstanceCount()), so a group never
// straddles two different tree positions. Stride=1 (every existing scalar
// caller) makes every division here a no-op, so this is a strictly
// backward-compatible extension.

// Row groups of one ordering, one call per group with (target = the group,
// source = Source's index for it) -- forward or backward -- or, scanning,
// one per (group, real row of a second layout) pair (see Scan). The only
// things that distinguish a tree, linear, linked, segment, scanning or pair
// pass are Source, Range and Scan; the loop is the same, and all three are
// resolved at compile time, so each costs exactly what its own
// hand-written loop did. Rows is whatever
// the Step call hands first after the view: a JointTopology, or
// (SelfSource only) a bare row count -- so a linear pass can share a Step
// call with a tree pass and its topology.
template <
    Direction Dir,
    size_t Stride,
    typename Source,
    typename Range = AllRows,
    typename Scan = NoScan>
  requires(
      (Stride == 1 || Source::kBatchable) && (Stride == 1 || Scan::kBatchable)
  )
struct RowTraversal {
  static constexpr Direction kDirection = Dir;
  static constexpr size_t kStride = Stride;

  template <typename Callable, typename Rows, typename... Rest>
    requires(Dir == Direction::kForward)
  static constexpr void Apply(
      const Callable& callable, const Rows& rows, const Rest&... rest
  ) {
    const size_t end = Range::template End<Stride>(rows);
    for (size_t j = Range::template Begin<Stride>(rows); j < end; ++j) {
      Scan::template Visit<Stride>(
          callable, rows, j, Source::template Of<Stride>(rows, j), rest...
      );
    }
  }

  template <typename Callable, typename Rows, typename... Rest>
    requires(Dir == Direction::kBackward)
  static constexpr void Apply(
      const Callable& callable, const Rows& rows, const Rest&... rest
  ) {
    const size_t begin = Range::template Begin<Stride>(rows);
    for (size_t j = Range::template End<Stride>(rows); j-- > begin;) {
      Scan::template Visit<Stride>(
          callable, rows, j, Source::template Of<Stride>(rows, j), rest...
      );
    }
  }

  template <typename Callable, typename Rows, typename... Rest>
  static constexpr void InitOp(
      const Callable& callable, const Rows& rows, const Rest&... rest
  ) {
    if (Range::template Begin<Stride>(rows) <
            Range::template End<Stride>(rows) &&
        Scan::HasWork(rest...)) {
      callable.Initialize(Source::template InitRow<Stride>(rows));
    }
  }
};

// Each joint after its parent (forward) or before it (backward), handed
// that parent's group as ArgIndex::kParent.
template <Direction Dir, size_t Stride>
using TreeTraversal = RowTraversal<Dir, Stride, ParentSource>;

// Every row on its own, handed itself as the source -- so a parentless
// field (the only kind that makes sense here) reads/writes itself either
// way.
template <Direction Dir, size_t Stride>
using LinearTraversal = RowTraversal<Dir, Stride, SelfSource>;

// Every entry of a list ordering, handed its link as ArgIndex::kLink.
using LinkedTraversal = RowTraversal<Direction::kForward, 1, LinkSource>;

// Only segment K's rows, forward, each handed its source -- by default its
// link (ArgIndex::kLink) -- so a pass can run an Op written for one list's
// entries over exactly those rows.
template <size_t K, typename Source = LinkSource>
using SegmentTraversal =
    RowTraversal<Direction::kForward, 1, Source, Segment<K>>;

// The per-segment traversals SegmentOps (algorithm_step.hpp) builds passes
// from, by segment index: each row of segment K handed its link or itself,
// or paired against every real row of a second layout (ScanRows).
template <size_t K>
using LinkedSegmentTraversal = SegmentTraversal<K, LinkSource>;
template <size_t K>
using SelfSegmentTraversal = SegmentTraversal<K, SelfSource>;
template <size_t K>
using LinkedSegmentScanTraversal =
    RowTraversal<Direction::kForward, 1, LinkSource, Segment<K>, ScanRows>;

// Every lane group of a pair ordering (topology::PairOrdering), handed its
// source (ArgIndex::kSource, broadcast), its partners (ArgIndex::kPartner)
// and which lanes are pairs (ArgIndex::kStructure) -- the pairs a broad
// phase, narrow phase or contact solver walks. A Step over pairs is given
// the pair topology after the rows (the other passes' topology), since a
// pass over pairs usually shares a Step with passes over the bodies they
// pair; this walks that second argument as its rows.
template <Direction Dir, size_t Stride>
struct PairTraversal {
  static constexpr Direction kDirection = Dir;
  static constexpr size_t kStride = Stride;
  using Walk = RowTraversal<Dir, Stride, PairSource, AllRows, PairedRows>;

  template <typename Callable, typename Rows, typename... Rest>
  static constexpr void Apply(
      const Callable& callable,
      const Rows& /*rows*/,
      const domain::PairTopology& pairs,
      const Rest&... /*rest*/
  ) {
    Walk::Apply(callable, pairs);
  }

  template <typename Callable, typename Rows, typename... Rest>
  static constexpr void InitOp(
      const Callable& callable,
      const Rows& /*rows*/,
      const domain::PairTopology& pairs,
      const Rest&... /*rest*/
  ) {
    Walk::InitOp(callable, pairs);
  }
};

using ForwardTreeTraversal = TreeTraversal<Direction::kForward, 1>;
using BackwardTreeTraversal = TreeTraversal<Direction::kBackward, 1>;
using ForwardLinearTraversal = LinearTraversal<Direction::kForward, 1>;
using BackwardLinearTraversal = LinearTraversal<Direction::kBackward, 1>;
static_assert(
    TraversalLike<ForwardTreeTraversal>,
    "ForwardTreeTraversal must satisfy TraversalLike concept"
);
static_assert(
    TraversalLike<BackwardTreeTraversal>,
    "BackwardTreeTraversal must satisfy TraversalLike concept"
);
static_assert(
    TraversalLike<ForwardLinearTraversal>,
    "ForwardLinearTraversal must satisfy TraversalLike concept"
);
static_assert(
    TraversalLike<BackwardLinearTraversal>,
    "BackwardLinearTraversal must satisfy TraversalLike concept"
);
static_assert(
    TraversalLike<LinkedTraversal>,
    "LinkedTraversal must satisfy TraversalLike concept"
);
static_assert(
    TraversalLike<SegmentTraversal<0>>,
    "SegmentTraversal must satisfy TraversalLike concept"
);
static_assert(
    TraversalLike<LinkedSegmentScanTraversal<0>>,
    "LinkedSegmentScanTraversal must satisfy TraversalLike concept"
);

// One call, at index 0 -- for an Op whose work is a single value the whole
// sim shares rather than anything per row: a scalar Op over
// topology::GlobalIndexing fields, e.g. an iterative solver turning the
// sums a reduction pass just left in them into its next step length. Takes
// (and ignores) whatever the Step call's other traversals take, so it sits
// in the same Ops list as the passes that reduce into those fields and the
// ones that read the result. Always Stride 1: there's only the one element
// to read. Initialize, if the Op has one, runs at that same index first.
struct GlobalTraversal {
  static constexpr Direction kDirection = Direction::kForward;
  static constexpr size_t kStride = 1;

  template <typename Callable, typename... Rest>
  static constexpr void Apply(const Callable& callable, const Rest&...) {
    callable(0, 0);
  }

  template <typename Callable, typename... Rest>
  static constexpr void InitOp(const Callable& callable, const Rest&...) {
    callable.Initialize(0);
  }
};
static_assert(
    TraversalLike<GlobalTraversal>,
    "GlobalTraversal must satisfy TraversalLike concept"
);

using ForwardPairTraversal = PairTraversal<Direction::kForward, 1>;
using BackwardPairTraversal = PairTraversal<Direction::kBackward, 1>;
static_assert(
    TraversalLike<ForwardPairTraversal>,
    "ForwardPairTraversal must satisfy TraversalLike concept"
);
static_assert(
    TraversalLike<BackwardPairTraversal>,
    "BackwardPairTraversal must satisfy TraversalLike concept"
);

}  // namespace achilles::engine::pass
