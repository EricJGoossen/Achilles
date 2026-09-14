#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include "domain/topology/topology_contract.hpp"
#include "engine/traversals.hpp"

using achilles::engine::BackwardLinearTraversal;
using achilles::engine::BackwardTreeTraversal;
using achilles::engine::Direction;
using achilles::engine::ForwardLinearTraversal;
using achilles::engine::ForwardTreeTraversal;
using achilles::engine::TraversalLike;

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
static_assert(achilles::domain::topology::TopologyLike<TopologyArchetype>);

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

  ASSERT_EQ(calls.size(), 3u);
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

  ASSERT_EQ(calls.size(), 3u);
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
  EXPECT_EQ(forward_seen, 42u);

  size_t backward_seen = 999;
  BackwardTreeTraversal::InitOp(RecordingInitOp{&backward_seen}, topology);
  EXPECT_EQ(backward_seen, 42u);
}

// LinearTraversal::Apply calls its callable with the same index twice --
// (target, parent) both equal to the bare index, per its own doc comment
// ("a parentless field... reads/writes itself either way") -- the shape
// OpInvoker::operator()(target_index, parent_index) needs, since Pass
// (algorithm_step.hpp) always pairs a Traversal with a real OpInvoker.
// See engine_algorithm_step.cpp for that pairing exercised for real.
TEST(LinearTraversalApply, ForwardVisitsEachIndexOnceInOrderTargetEqualsParent) {
  std::vector<std::pair<size_t, size_t>> calls;
  ForwardLinearTraversal::Apply(
      [&](size_t target, size_t parent) { calls.emplace_back(target, parent); }, 3
  );
  EXPECT_EQ(
      calls,
      (std::vector<std::pair<size_t, size_t>>{{0, 0}, {1, 1}, {2, 2}})
  );
}

TEST(LinearTraversalApply, BackwardVisitsEachIndexOnceInReverseOrder) {
  std::vector<std::pair<size_t, size_t>> calls;
  BackwardLinearTraversal::Apply(
      [&](size_t target, size_t parent) { calls.emplace_back(target, parent); }, 3
  );
  EXPECT_EQ(
      calls,
      (std::vector<std::pair<size_t, size_t>>{{2, 2}, {1, 1}, {0, 0}})
  );
}

TEST(LinearTraversalInitOp, AlwaysCallsWithZero) {
  size_t seen = 999;
  ForwardLinearTraversal::InitOp(RecordingInitOp{&seen});
  EXPECT_EQ(seen, 0u);
}

TEST(TraversalLikeConcept, AllFourTraversalsSatisfyIt) {
  EXPECT_TRUE(TraversalLike<ForwardTreeTraversal>);
  EXPECT_TRUE(TraversalLike<BackwardTreeTraversal>);
  EXPECT_TRUE(TraversalLike<ForwardLinearTraversal>);
  EXPECT_TRUE(TraversalLike<BackwardLinearTraversal>);
}
