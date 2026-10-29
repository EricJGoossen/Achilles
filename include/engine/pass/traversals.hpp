#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <type_traits>
#include <utility>

#include "domain/joint_topology.hpp"
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
    requires(
        void (&callable)(size_t, size_t), detail::TraversalProbe topology
    ) {
      { T::Apply(callable, topology) } -> std::same_as<void>;
    } &&
    requires(void (&callable)(size_t), detail::TraversalProbe topology) {
      { T::InitOp(callable, topology) } -> std::same_as<void>;
    };

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

// -- Sources: where a RowTraversal's second index (ArgIndex::kSource) comes
// from for each row group j, and where Initialize runs. Each is a pair of
// static functions resolved at compile time, plus whether it can drive a
// batched (Stride > 1) pass.

// The row itself -- a pass whose fields all live in one row space reads
// and writes each row on its own. Driven by a bare row count or anything
// with a Size(). Initialize runs at row 0.
struct SelfSource {
  static constexpr bool kBatchable = true;

  template <size_t Stride, typename Rows>
  static constexpr size_t Of(const Rows& /*rows*/, size_t group) {
    return group;
  }
  template <size_t Stride, typename Rows>
  static constexpr size_t InitRow(const Rows& /*rows*/) {
    return 0;
  }
};

// The row's parent group, in the same row space (ArgIndex::kParent) -- a
// tree ordering's topology. Batchable because the tree planner guarantees
// no parent shares a lane group with its child (see JointTopology::
// IsBatchSafe). Initialize runs at row 0's parent: the base row a root
// joint reads as its parent, seeded before the walk.
struct ParentSource {
  static constexpr bool kBatchable = true;

  template <size_t Stride, domain::TopologyLike Topology>
  static constexpr size_t Of(const Topology& parents, size_t group) {
    return parents[group * Stride] / Stride;
  }
  template <size_t Stride, domain::TopologyLike Topology>
  static constexpr size_t InitRow(const Topology& parents) {
    return parents[0] / Stride;
  }
};

// The row's *link* (ArgIndex::kLink) -- for a list ordering (topology::
// ListOrdering), whose topology holds, per row, the row in another
// ordering's row space that entry is attached to (e.g. the joint a sensor
// is mounted on). An Op reads that other ordering's fields at kLink and its
// own list-ordered ones at kTarget. Not batchable: links point anywhere, so
// a lane group of entries has no single source group to load. Initialize
// runs at row 0 of the list itself.
struct LinkSource {
  static constexpr bool kBatchable = false;

  template <size_t Stride, domain::TopologyLike Topology>
  static constexpr size_t Of(const Topology& links, size_t row) {
    return links[row];
  }
  template <size_t Stride, domain::TopologyLike Topology>
  static constexpr size_t InitRow(const Topology& /*links*/) {
    return 0;
  }
};

namespace detail {

// How many raw rows a RowTraversal walks: a bare count as-is, or a
// topology's Size().
constexpr size_t RowCount(size_t rows) { return rows; }
template <typename Rows>
  requires requires(const Rows& rows) { rows.Size(); }
constexpr size_t RowCount(const Rows& rows) {
  return rows.Size();
}

}  // namespace detail

// Which row groups a RowTraversal walks, [Begin, End) in groups of Stride:
// every one (AllRows), or only segment K of a topology split into segments
// (one per list of a topology::ListOrdering -- see
// domain::JointTopology::SegmentBegin), one row at a time since a segment
// needn't start on a lane boundary.
struct AllRows {
  template <size_t Stride, typename Rows>
  static constexpr size_t Begin(const Rows& /*rows*/) {
    return 0;
  }
  template <size_t Stride, typename Rows>
  static constexpr size_t End(const Rows& rows) {
    return detail::RowCount(rows) / Stride;
  }
};

template <size_t K>
struct Segment {
  template <size_t Stride, typename Rows>
    requires(Stride == 1)
  static constexpr size_t Begin(const Rows& rows) {
    return rows.SegmentBegin(K);
  }
  template <size_t Stride, typename Rows>
    requires(Stride == 1)
  static constexpr size_t End(const Rows& rows) {
    return rows.SegmentEnd(K);
  }
};

// What a RowTraversal does at each row group it visits: call once with
// (group, source) -- NoScan -- or, ScanRows, pair the group with every real
// row of the JointLayout a Step was given second (its padding rows, which
// hold no real entry, skipped), calling once per pair with that row as a
// third index (ArgIndex::kScene). Scanning is for an Op relating each entry
// of one list to every row of another -- a lidar against every body's
// shape, say -- that still reads the entry's own source (its link). Given
// no second argument, a scan has nothing to pair against and does nothing.
struct NoScan {
  template <typename Callable, typename... Rest>
  static constexpr void Visit(
      const Callable& callable,
      size_t group,
      size_t source,
      const Rest&... /*rest*/
  ) {
    callable(group, source);
  }
  template <typename... Rest>
  static constexpr bool HasWork(const Rest&... /*rest*/) {
    return true;
  }
};

struct ScanRows {
  template <typename Callable, typename Scene, typename... Rest>
  static constexpr void Visit(
      const Callable& callable,
      size_t group,
      size_t source,
      const Scene& scene,
      const Rest&... /*rest*/
  ) {
    const size_t rows = scene.PaddedSize();
    for (size_t row = 0; row < rows; ++row) {
      if (!scene.IsPadding(row)) {
        callable(group, source, row);
      }
    }
  }
  template <typename Callable>
  static constexpr void Visit(
      const Callable& /*callable*/, size_t /*group*/, size_t /*source*/
  ) {}

  template <typename Scene, typename... Rest>
  static constexpr bool HasWork(const Scene& scene, const Rest&... /*rest*/) {
    return scene.PaddedSize() > 0;
  }
  static constexpr bool HasWork() { return false; }
};

// Row groups of one ordering, one call per group with (target = the group,
// source = Source's index for it) -- forward or backward -- or, scanning,
// one per (group, real row of a second layout) pair (see Scan). The only
// things that distinguish a tree, linear, linked, segment or scanning pass
// are Source, Range and Scan; the loop is the same, and all three are
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
      (Stride == 1 || Source::kBatchable) &&
      (Stride == 1 || std::is_same_v<Scan, NoScan>)
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
      Scan::Visit(callable, j, Source::template Of<Stride>(rows, j), rest...);
    }
  }

  template <typename Callable, typename Rows, typename... Rest>
    requires(Dir == Direction::kBackward)
  static constexpr void Apply(
      const Callable& callable, const Rows& rows, const Rest&... rest
  ) {
    const size_t begin = Range::template Begin<Stride>(rows);
    for (size_t j = Range::template End<Stride>(rows); j-- > begin;) {
      Scan::Visit(callable, j, Source::template Of<Stride>(rows, j), rest...);
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

// What a PairTraversal knows about one (source row, target group) call
// from its own topology alone, handed to an Op through its tuple's
// ArgIndex::kStructure slot (see kTraversalInputs, engine/op_contract.hpp):
// bit t of distinct_lanes is set when lane t's target row is a pair worth
// testing at all -- above the source row (so each unordered pair appears
// exactly once, and never a row against itself), and neither row the
// other's parent (a joint's two bodies always touch where they meet).
// Everything else about whether the pair matters (geometry, filtering) is
// the Op's own business.
struct PairStructure {
  std::uint64_t distinct_lanes;
};

// Every source row against every lane group of targets -- the all-pairs
// walk a broad phase brute-forces. The callable gets (target_group,
// BroadcastIndex{source_row}, PairStructure): a row-indexed argument
// naming ArgIndex::kSource reads source_row's own value splatted across
// every lane, so one call compares that one row against a whole lane group
// of targets at once; a PairGroupIndexing<Stride> field (see
// topology/indexing_policy.hpp) addresses the one (source_row,
// target_group) element summarizing that whole call, and a
// PairGroupIndexing<Stride, Stride> field that call's own batch of
// per-pair values, lane for lane with the targets; and kStructure says
// which of those lanes are real, distinct, non-parent/child pairs.
//
// Driven by the topology, like TreeTraversal: it's what PairStructure is
// computed from (each row's parent), and its Size() -- every real and
// padding row, but not the reserved base row or lane filler -- is how many
// rows are walked.
//
// Only the upper triangle, by group: source row s is paired with target
// groups s / Stride onward, never the groups wholly before its own. Within
// s's own group, the lanes at or below s are still visited, with their
// PairStructure bits clear. Elements of a PairGroupIndexing output whose
// target group comes wholly before its source's are never written, so they
// keep whatever Carve/SeedPadding left there.
template <Direction Dir, size_t Stride>
struct PairTraversal {
  static constexpr Direction kDirection = Dir;
  static constexpr size_t kStride = Stride;

  template <typename Callable, domain::TopologyLike Topology, typename... Rest>
    requires(Dir == Direction::kForward)
  static constexpr void Apply(
      const Callable& callable, const Topology& joints, const Rest&...
  ) {
    size_t size = joints.Size();
    size_t groups = size / Stride;
    for (size_t source = 0; source < size; ++source) {
      for (size_t j = source / Stride; j < groups; ++j) {
        Visit(callable, joints, source, j);
      }
    }
  }

  template <typename Callable, domain::TopologyLike Topology, typename... Rest>
    requires(Dir == Direction::kBackward)
  static constexpr void Apply(
      const Callable& callable, const Topology& joints, const Rest&...
  ) {
    size_t size = joints.Size();
    size_t groups = size / Stride;
    for (size_t source = size; source-- > 0;) {
      for (size_t j = groups; j-- > source / Stride;) {
        Visit(callable, joints, source, j);
      }
    }
  }

  // Initialize at index 0 if there's anything to walk at all, the same as
  // LinearTraversal's.
  template <typename Callable, domain::TopologyLike Topology, typename... Rest>
  static constexpr void InitOp(
      const Callable& callable, const Topology& joints, const Rest&...
  ) {
    if (joints.Size() / Stride > 0) {
      callable.Initialize(0);
    }
  }

 private:
  template <typename Callable, domain::TopologyLike Topology>
  static constexpr void Visit(
      const Callable& callable,
      const Topology& joints,
      size_t source,
      size_t group
  ) {
    callable(
        group,
        topology::BroadcastIndex{source},
        Structure(joints, source, group)
    );
  }

  // t > s && parent(t) != s && parent(s) != t, one bit per lane. A padding
  // row's parent is the reserved base row, which is never a walked row, so
  // padding never reads as anyone's parent or child.
  template <domain::TopologyLike Topology>
  static constexpr PairStructure Structure(
      const Topology& joints, size_t source, size_t group
  ) {
    size_t source_parent = joints[source];
    std::uint64_t lanes = 0;
    for (size_t lane = 0; lane < Stride; ++lane) {
      size_t target = (group * Stride) + lane;
      bool distinct = target > source && joints[target] != source &&
                      source_parent != target;
      lanes |= std::uint64_t{distinct} << lane;
    }
    return PairStructure{lanes};
  }
};
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
