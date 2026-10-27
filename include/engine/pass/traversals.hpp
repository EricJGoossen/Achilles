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
// see Layout::PaddedSize()/AllocateTopology), but OpInvoker feeds whatever
// index this traversal hands it straight into View::Load/Store<T>, which
// for a batched T addresses storage in units of Stride-sized *groups*, not
// one raw row at a time (see View's own comment on Value/Load: "instance
// index directly when T is scalar, batch index when T is batched"). So a
// traversal driving a batched Op must walk Size()/Stride groups, not
// Size() raw rows, and must divide each raw-row parent value by Stride too
// -- both rows of a Stride-sized batch group always resolve to the same
// parent GROUP, since SimAllocator always pads every level's own per-
// instance block out to a multiple of the widest Stride any hosted field
// needs (see topology::Layout::ViewInstanceCount()), so a group never
// straddles two different tree positions. Stride=1 (every existing scalar
// caller) makes every division here a no-op, so this is a strictly
// backward-compatible extension.
template <Direction Dir, size_t Stride>
struct TreeTraversal {
  static constexpr Direction kDirection = Dir;
  static constexpr size_t kStride = Stride;

  template <typename Callable, domain::TopologyLike Topology, typename... Rest>
  static constexpr void Apply(
      const Callable& callable, const Topology& topology, const Rest&...
  ) {
    size_t groups = topology.Size() / Stride;
    if constexpr (Dir == Direction::kForward) {
      for (size_t j = 0; j < groups; ++j) {
        callable(j, topology[j * Stride] / Stride);
      }
    } else {
      for (size_t j = groups; j-- > 0;) {
        callable(j, topology[j * Stride] / Stride);
      }
    }
  }

  template <typename Callable, typename Topology, typename... Rest>
  static constexpr void InitOp(
      const Callable& callable, const Topology& topology, const Rest&...
  ) {
    if (topology.Size() > 0) {
      callable.Initialize(topology[0] / Stride);
    }
  }
};
using ForwardTreeTraversal = TreeTraversal<Direction::kForward, 1>;
using BackwardTreeTraversal = TreeTraversal<Direction::kBackward, 1>;
static_assert(
    TraversalLike<ForwardTreeTraversal>,
    "ForwardTreeTraversal must satisfy TraversalLike concept"
);
static_assert(
    TraversalLike<BackwardTreeTraversal>,
    "BackwardTreeTraversal must satisfy TraversalLike concept"
);

// Accepts either a bare size, or anything with a Size() (a JointTopology
// works here too) -- so a linear pass can be driven by the same topology
// argument a tree pass in the same Step call uses. LinearTraversal itself
// only ever hands back one index (there's no parent to also report), while
// an OpInvoker always expects (target_index, parent_index) -- Apply bridges
// that by calling the callable with the same index twice, so a parentless
// field (the only kind that makes sense on a linear pass) reads/writes
// itself either way.
//
// Stride -- same meaning and same backward-compatible default as
// TreeTraversal's own (see its comment): `size` is always a raw-row/raw-
// instance count, and a batched Op needs size/Stride groups, not size raw
// steps. There's no separate parent value to divide here (target and
// parent are always the same group index already).
template <Direction Dir, size_t Stride>
struct LinearTraversal {
  static constexpr Direction kDirection = Dir;
  static constexpr size_t kStride = Stride;

  template <typename Callable, typename SizeOrTopology, typename... Rest>
  static constexpr void Apply(
      const Callable& callable,
      const SizeOrTopology& size_or_topology,
      const Rest&...
  ) {
    size_t size = [&] {
      if constexpr (requires { size_or_topology.Size(); }) {
        return size_or_topology.Size();
      } else {
        return size_or_topology;
      }
    }();
    size_t groups = size / Stride;
    if constexpr (Dir == Direction::kForward) {
      for (size_t j = 0; j < groups; ++j) {
        callable(j, j);
      }
    } else {
      for (size_t j = groups; j-- > 0;) {
        callable(j, j);
      }
    }
  }

  template <typename Callable, typename SizeOrTopology, typename... Rest>
  static constexpr void InitOp(
      const Callable& callable,
      const SizeOrTopology& size_or_topology,
      const Rest&...
  ) {
    size_t size = [&] {
      if constexpr (requires { size_or_topology.Size(); }) {
        return size_or_topology.Size();
      } else {
        return size_or_topology;
      }
    }();
    if (size / Stride > 0) {
      callable.Initialize(0);
    }
  }
};
using ForwardLinearTraversal = LinearTraversal<Direction::kForward, 1>;
using BackwardLinearTraversal = LinearTraversal<Direction::kBackward, 1>;
static_assert(
    TraversalLike<ForwardLinearTraversal>,
    "ForwardLinearTraversal must satisfy TraversalLike concept"
);
static_assert(
    TraversalLike<BackwardLinearTraversal>,
    "BackwardLinearTraversal must satisfy TraversalLike concept"
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
