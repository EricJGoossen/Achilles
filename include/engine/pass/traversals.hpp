#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <type_traits>
#include <utility>

#include "domain/joint_topology.hpp"

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
// static `Apply`/`ApplyToBase` entry points shaped like the two traversals
// below -- Apply drives a callable over every (target, parent) index pair
// (forward or backward), ApplyToBase calls the callable once, with just the
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

}  // namespace achilles::engine::pass
