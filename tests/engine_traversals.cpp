#include <gtest/gtest.h>

#include <cstddef>
#include <utility>
#include <vector>

#include "domain/joint_topology.hpp"
#include "engine/pass/traversals.hpp"

using achilles::engine::pass::BackwardLinearTraversal;
using achilles::engine::pass::BackwardTreeTraversal;
using achilles::engine::pass::Direction;
using achilles::engine::pass::ForwardLinearTraversal;
using achilles::engine::pass::ForwardTreeTraversal;
using achilles::engine::pass::TraversalLike;

namespace {

// Minimal type satisfying exactly what TreeTraversal::Apply needs from
// its second argument (Size() + operator[], i.e. TopologyLike) -- used
// here instead of a real JointTopology (domain/topology/joint_topology.hpp)
// so these tests describe *only* how TreeTraversal walks it, free of
// JointTopology's own segment/batch-safety machinery (already covered in
// domain_topology_joint_topology.cpp).
struct TopologyArchetype {
  std::vector<size_t> parents;

  size_t Size() const { return parents.size(); }
  size_t operator[](size_t i) const { return parents[i]; }
};
static_assert(achilles::domain::TopologyLike<TopologyArchetype>);

// Has a Size() but, unlike a bare size_t, no implicit conversion to
// size_t -- the shape a real JointTopology has (Size() + operator[], no
// operator size_t()). Used to prove LinearTraversal::Apply actually
// accepts "anything with a Size()" the way its own doc comment promises,
// not just a literal size_t.
struct SizeOnlyArchetype {
  size_t count;
  size_t Size() const { return count; }
};

// InitOp calls `op.Initialize(...)` by name (see TreeTraversal::InitOp/
// LinearTraversal::InitOp in engine/traversals.hpp) -- unlike Apply, which
// takes any two/one-argument callable, InitOp specifically requires an
// object with an Initialize method, so a bare lambda won't do here.
struct RecordingInitOp {
  size_t* seen;
  void Initialize(size_t base_index) const { *seen = base_index; }
};

}  // namespace

TEST(TraversalDirectionMetadata, KDirectionMatchesTheTemplateArgument) {
  EXPECT_EQ(ForwardTreeTraversal::kDirection, Direction::kForward);
  EXPECT_EQ(BackwardTreeTraversal::kDirection, Direction::kBackward);
  EXPECT_EQ(ForwardLinearTraversal::kDirection, Direction::kForward);
  EXPECT_EQ(BackwardLinearTraversal::kDirection, Direction::kBackward);
}

// TreeTraversal::Apply calls its callable with (target_index,
// topology[target_index]) for every index, 0..Size()-1 forward or the
// reverse order backward -- this is the two-argument shape
// OpInvoker::operator()(target_index, parent_index) expects (see
// engine_algorithm_step.cpp for that pairing exercised for real).
TEST(TreeTraversalApply, ForwardVisitsEachIndexOnceInOrderWithItsParent) {
  TopologyArchetype topology{{10, 0, 1}};  // joint i's parent = parents[i]
  std::vector<std::pair<size_t, size_t>> calls;

  ForwardTreeTraversal::Apply(
      [&](size_t target, size_t parent) { calls.emplace_back(target, parent); },
      topology
  );

  ASSERT_EQ(calls.size(), 3U);
  EXPECT_EQ(calls[0], (std::pair<size_t, size_t>{0, 10}));
  EXPECT_EQ(calls[1], (std::pair<size_t, size_t>{1, 0}));
  EXPECT_EQ(calls[2], (std::pair<size_t, size_t>{2, 1}));
}

TEST(TreeTraversalApply, BackwardVisitsEachIndexOnceInReverseOrder) {
  TopologyArchetype topology{{10, 0, 1}};
  std::vector<std::pair<size_t, size_t>> calls;

  BackwardTreeTraversal::Apply(
      [&](size_t target, size_t parent) { calls.emplace_back(target, parent); },
      topology
  );

  ASSERT_EQ(calls.size(), 3U);
  EXPECT_EQ(calls[0], (std::pair<size_t, size_t>{2, 1}));
  EXPECT_EQ(calls[1], (std::pair<size_t, size_t>{1, 0}));
  EXPECT_EQ(calls[2], (std::pair<size_t, size_t>{0, 10}));
}

// InitOp always calls Initialize with topology[0] alone, for either
// direction -- there is exactly one "base" row per topology, independent
// of which way a later real pass over it walks.
TEST(TreeTraversalInitOp, AlwaysUsesTopologyIndexZero) {
  TopologyArchetype topology{{42, 0, 1}};

  size_t forward_seen = 999;
  ForwardTreeTraversal::InitOp(RecordingInitOp{&forward_seen}, topology);
  EXPECT_EQ(forward_seen, 42U);

  size_t backward_seen = 999;
  BackwardTreeTraversal::InitOp(RecordingInitOp{&backward_seen}, topology);
  EXPECT_EQ(backward_seen, 42U);
}

// An empty topology has no row 0 to initialize -- topology[0] would be
// out of range, so InitOp must skip calling Initialize rather than
// indexing into an empty topology.
TEST(TreeTraversalInitOp, SkipsInitializeWhenTopologyIsEmpty) {
  TopologyArchetype topology{{}};

  size_t seen = 999;
  ForwardTreeTraversal::InitOp(RecordingInitOp{&seen}, topology);
  EXPECT_EQ(seen, 999U);
}

// Backward Apply's loop is `for (size_t j = size; j-- > 0;)` -- with
// size == 0, the comparison is false before any decrement is observed, so
// no call happens, but this relies on evaluation order rather than an
// explicit `size == 0` guard. Locks in that this stays call-free (rather
// than, say, wrapping around and iterating size_t's full range) if that
// loop is ever rewritten.
TEST(TreeTraversalApply, EmptyTopologyMakesNoCalls) {
  TopologyArchetype topology{{}};
  std::vector<std::pair<size_t, size_t>> calls;

  ForwardTreeTraversal::Apply(
      [&](size_t target, size_t parent) { calls.emplace_back(target, parent); },
      topology
  );
  BackwardTreeTraversal::Apply(
      [&](size_t target, size_t parent) { calls.emplace_back(target, parent); },
      topology
  );

  EXPECT_TRUE(calls.empty());
}

// LinearTraversal::Apply calls its callable with the same index twice --
// (target, parent) both equal to the bare index, per its own doc comment
// ("a parentless field... reads/writes itself either way") -- the shape
// OpInvoker::operator()(target_index, parent_index) needs, since Pass
// (algorithm_step.hpp) always pairs a Traversal with a real OpInvoker.
// See engine_algorithm_step.cpp for that pairing exercised for real.
TEST(
    LinearTraversalApply, ForwardVisitsEachIndexOnceInOrderTargetEqualsParent
) {
  std::vector<std::pair<size_t, size_t>> calls;
  ForwardLinearTraversal::Apply(
      [&](size_t target, size_t parent) { calls.emplace_back(target, parent); },
      3
  );
  EXPECT_EQ(
      calls, (std::vector<std::pair<size_t, size_t>>{{0, 0}, {1, 1}, {2, 2}})
  );
}

TEST(LinearTraversalApply, BackwardVisitsEachIndexOnceInReverseOrder) {
  std::vector<std::pair<size_t, size_t>> calls;
  BackwardLinearTraversal::Apply(
      [&](size_t target, size_t parent) { calls.emplace_back(target, parent); },
      3
  );
  EXPECT_EQ(
      calls, (std::vector<std::pair<size_t, size_t>>{{2, 2}, {1, 1}, {0, 0}})
  );
}

// Regression test: LinearTraversal::Apply's second parameter used to be
// typed as a literal size_t, so a real JointTopology (Size() + operator[],
// no operator size_t()) couldn't be passed to it at all despite the
// surrounding docs promising "anything with a Size()" -- this only
// compiles now that Apply dispatches on whether its argument has Size().
TEST(LinearTraversalApply, AcceptsATopologyLikeSizeArgument) {
  std::vector<std::pair<size_t, size_t>> calls;
  ForwardLinearTraversal::Apply(
      [&](size_t target, size_t parent) { calls.emplace_back(target, parent); },
      SizeOnlyArchetype{3}
  );
  EXPECT_EQ(
      calls, (std::vector<std::pair<size_t, size_t>>{{0, 0}, {1, 1}, {2, 2}})
  );
}

// Same evaluation-order concern as TreeTraversalApply's empty case, for
// LinearTraversal's identically-shaped `for (size_t j = size; j-- > 0;)`
// loop.
TEST(LinearTraversalApply, ZeroSizeMakesNoCalls) {
  std::vector<std::pair<size_t, size_t>> calls;

  ForwardLinearTraversal::Apply(
      [&](size_t target, size_t parent) { calls.emplace_back(target, parent); },
      0
  );
  BackwardLinearTraversal::Apply(
      [&](size_t target, size_t parent) { calls.emplace_back(target, parent); },
      0
  );

  EXPECT_TRUE(calls.empty());
}

TEST(LinearTraversalInitOp, CallsWithZeroWhenNonEmpty) {
  size_t seen = 999;
  ForwardLinearTraversal::InitOp(RecordingInitOp{&seen}, 3);
  EXPECT_EQ(seen, 0U);
}

// A zero-length linear pass has no row 0 to initialize -- Apply itself
// would run zero iterations, so InitOp must skip calling Initialize rather
// than unconditionally touching index 0.
TEST(LinearTraversalInitOp, SkipsInitializeWhenEmpty) {
  size_t seen = 999;
  ForwardLinearTraversal::InitOp(RecordingInitOp{&seen}, 0);
  EXPECT_EQ(seen, 999U);
}

TEST(TraversalLikeConcept, AllFourTraversalsSatisfyIt) {
  EXPECT_TRUE(TraversalLike<ForwardTreeTraversal>);
  EXPECT_TRUE(TraversalLike<BackwardTreeTraversal>);
  EXPECT_TRUE(TraversalLike<ForwardLinearTraversal>);
  EXPECT_TRUE(TraversalLike<BackwardLinearTraversal>);
}

// ---------------------------------------------------------------------------
// Stride: a batched Op's real, memory-backed topology is always sized in
// raw rows (Layout::PaddedSize()), but OpInvoker feeds a traversal's index
// straight into a batched View::Load/Store, which addresses storage in
// units of Stride-sized groups (see TreeTraversal's own comment). Every
// test above uses the default Stride=1, where group == raw row -- these
// exercise Stride > 1 directly against TreeTraversal/LinearTraversal in
// isolation, independent of any real View/SimAllocator, at every lane size
// TESTING.md 9.4 requires (4, 8, 32).
// ---------------------------------------------------------------------------

static_assert(achilles::engine::pass::TreeTraversal<Direction::kForward>::kStride == 1);
static_assert(
    achilles::engine::pass::LinearTraversal<Direction::kForward>::kStride == 1
);

namespace {

using achilles::engine::pass::LinearTraversal;
using achilles::engine::pass::TreeTraversal;

// `groups` positions, each Stride raw rows wide: group g's own Stride rows
// all resolve to raw-row parent `parent_group * stride` -- the shape a real
// Layout always produces (every row within one Stride-sized group shares
// the same parent GROUP, never a mix -- see TreeTraversal's own comment on
// why Layout guarantees this).
std::vector<size_t> MakeGroupedParents(
    size_t groups, size_t stride, size_t base_parent_group
) {
  std::vector<size_t> parents(groups * stride);
  for (size_t g = 0; g < groups; ++g) {
    size_t parent_group = (g == 0) ? base_parent_group : g - 1;
    for (size_t lane = 0; lane < stride; ++lane) {
      parents[(g * stride) + lane] = parent_group * stride;
    }
  }
  return parents;
}

}  // namespace

TEST(TreeTraversalStride, ForwardDividesRawRowsIntoGroupsAtLaneFour) {
  constexpr size_t kStride = 4;
  TopologyArchetype topology{MakeGroupedParents(2, kStride, /*base_parent_group=*/10)};
  std::vector<std::pair<size_t, size_t>> calls;

  TreeTraversal<Direction::kForward, kStride>::Apply(
      [&](size_t target, size_t parent) { calls.emplace_back(target, parent); },
      topology
  );

  ASSERT_EQ(calls.size(), 2U);
  EXPECT_EQ(calls[0], (std::pair<size_t, size_t>{0, 10}));
  EXPECT_EQ(calls[1], (std::pair<size_t, size_t>{1, 0}));
}

TEST(TreeTraversalStride, ForwardDividesRawRowsIntoGroupsAtLaneEight) {
  constexpr size_t kStride = 8;
  TopologyArchetype topology{MakeGroupedParents(3, kStride, /*base_parent_group=*/99)};
  std::vector<std::pair<size_t, size_t>> calls;

  TreeTraversal<Direction::kForward, kStride>::Apply(
      [&](size_t target, size_t parent) { calls.emplace_back(target, parent); },
      topology
  );

  ASSERT_EQ(calls.size(), 3U);
  EXPECT_EQ(calls[0], (std::pair<size_t, size_t>{0, 99}));
  EXPECT_EQ(calls[1], (std::pair<size_t, size_t>{1, 0}));
  EXPECT_EQ(calls[2], (std::pair<size_t, size_t>{2, 1}));
}

TEST(TreeTraversalStride, ForwardDividesRawRowsIntoGroupsAtLaneThirtyTwo) {
  constexpr size_t kStride = 32;
  TopologyArchetype topology{MakeGroupedParents(2, kStride, /*base_parent_group=*/5)};
  std::vector<std::pair<size_t, size_t>> calls;

  TreeTraversal<Direction::kForward, kStride>::Apply(
      [&](size_t target, size_t parent) { calls.emplace_back(target, parent); },
      topology
  );

  ASSERT_EQ(calls.size(), 2U);
  EXPECT_EQ(calls[0], (std::pair<size_t, size_t>{0, 5}));
  EXPECT_EQ(calls[1], (std::pair<size_t, size_t>{1, 0}));
}

// Direction and Stride are independent knobs -- backward order still holds
// once Stride > 1 groups (rather than raw rows) are what's being reversed.
TEST(TreeTraversalStride, BackwardVisitsGroupsInReverseOrder) {
  constexpr size_t kStride = 8;
  TopologyArchetype topology{MakeGroupedParents(3, kStride, /*base_parent_group=*/99)};
  std::vector<std::pair<size_t, size_t>> calls;

  TreeTraversal<Direction::kBackward, kStride>::Apply(
      [&](size_t target, size_t parent) { calls.emplace_back(target, parent); },
      topology
  );

  ASSERT_EQ(calls.size(), 3U);
  EXPECT_EQ(calls[0], (std::pair<size_t, size_t>{2, 1}));
  EXPECT_EQ(calls[1], (std::pair<size_t, size_t>{1, 0}));
  EXPECT_EQ(calls[2], (std::pair<size_t, size_t>{0, 99}));
}

// InitOp seeds from topology[0] -- a raw row -- so it must divide by Stride
// too, the same as every real Apply step's own parent value.
TEST(TreeTraversalStride, InitOpDividesRawRowByStride) {
  constexpr size_t kStride = 4;
  TopologyArchetype topology{MakeGroupedParents(2, kStride, /*base_parent_group=*/10)};

  size_t seen = 999;
  TreeTraversal<Direction::kForward, kStride>::InitOp(
      RecordingInitOp{&seen}, topology
  );
  EXPECT_EQ(seen, 10U);
}

// LinearTraversal's own Stride: `size` is a raw count (e.g. a linear pool's
// own instance count), so Apply must walk size/Stride groups, not size raw
// steps -- the same grouping TreeTraversal applies to a topology's rows.
TEST(LinearTraversalStride, ForwardDividesSizeIntoGroups) {
  constexpr size_t kStride = 4;
  std::vector<std::pair<size_t, size_t>> calls;

  LinearTraversal<Direction::kForward, kStride>::Apply(
      [&](size_t target, size_t parent) { calls.emplace_back(target, parent); },
      3 * kStride
  );

  EXPECT_EQ(
      calls, (std::vector<std::pair<size_t, size_t>>{{0, 0}, {1, 1}, {2, 2}})
  );
}
