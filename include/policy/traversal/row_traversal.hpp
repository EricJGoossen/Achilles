// Row groups of one ordering, walked forward or backward: tree, linear,
// linked and segment traversals.

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
#include "policy/range/all_range.hpp"
#include "policy/range/base_range.hpp"
#include "policy/range/segment_range.hpp"
#include "policy/scan/base_scan.hpp"
#include "policy/scan/no_scan.hpp"
#include "policy/scan/pair_scan.hpp"
#include "policy/scan/row_scan.hpp"
#include "policy/source/base_source.hpp"
#include "policy/source/link_source.hpp"
#include "policy/source/pair_source.hpp"
#include "policy/source/parent_source.hpp"
#include "policy/source/self_source.hpp"
#include "policy/traversal/base_traversal.hpp"

namespace achilles::policy::traversal {

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
    source::SourceLike Source,
    range::RangeLike Range = range::AllRange,
    scan::ScanLike Scan = scan::NoScan>
  requires(
      (Stride == 1 || Source::kBatchable) && (Stride == 1 || Scan::kBatchable)
  )
struct RowTraversal : BaseTraversal<Dir, Stride> {
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
using TreeTraversal = RowTraversal<Dir, Stride, source::ParentSource>;

// Every row on its own, handed itself as the source -- so a parentless
// field (the only kind that makes sense here) reads/writes itself either
// way.
template <Direction Dir, size_t Stride>
using LinearTraversal = RowTraversal<Dir, Stride, source::SelfSource>;

// Every entry of a list ordering, handed its link as ArgIndex::kLink.
using LinkedTraversal =
    RowTraversal<Direction::kForward, 1, source::LinkSource>;

// Only segment K's rows, forward, each handed its source -- by default its
// link (ArgIndex::kLink) -- so a pass can run an Op written for one list's
// entries over exactly those rows.
template <size_t K, typename Source = source::LinkSource>
using SegmentTraversal =
    RowTraversal<Direction::kForward, 1, Source, range::SegmentRange<K>>;

// The per-segment traversals SegmentOps (algorithm_step.hpp) builds passes
// from, by segment index: each row of segment K handed its link or itself,
// or paired against every real row of a second layout (RowScan).
template <size_t K>
using LinkedSegmentTraversal = SegmentTraversal<K, source::LinkSource>;

template <size_t K>
using SelfSegmentTraversal = SegmentTraversal<K, source::SelfSource>;

template <size_t K>
using LinkedSegmentScanTraversal = RowTraversal<
    Direction::kForward,
    1,
    source::LinkSource,
    range::SegmentRange<K>,
    scan::RowScan>;

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

}  // namespace achilles::policy::traversal
