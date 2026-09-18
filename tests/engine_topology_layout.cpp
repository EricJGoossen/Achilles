#include <gtest/gtest.h>

#include <cstddef>
#include <span>
#include <utility>
#include <vector>

#include "domain/archetype.hpp"
#include "engine/memory/arena.hpp"
#include "engine/topology/layout.hpp"
#include "engine/topology/ordering_policy.hpp"

using achilles::domain::ArchetypeJointHandle;
using achilles::domain::ArchetypeTreeStructure;
using achilles::engine::memory::Arena;
using achilles::engine::topology::Layout;
using achilles::engine::topology::LinearOrdering;
using achilles::engine::topology::TopologicalOrdering;

namespace {

// One root archetype, THREE instances, a 3-joint tree per instance (joint
// 0 is the root; joints 1 and 2 are both direct children of joint 0).
// Ordering::Consolidate lays a topologically-ordered archetype out one
// *joint position* at a time -- every instance's own joint 0 first (padded
// to a lane multiple of the instance count), then every instance's own
// joint 1, then joint 2 -- rather than interleaving different joints of
// the same instance together. That's what makes the result batch-safe by
// construction: a lane group is always some instances' copies of the
// *same* joint, so a parent and its child (different joints) can never
// land in the same group regardless of how many instances there are.
//
// lane_size 2, 3 instances -> each joint's own block is
// RoundUpToLane(3, 2) = 4 rows wide: 3 real (one per instance) + 1
// padding. Three joints -> PaddedSize() == 12.
Layout BuildThreeInstanceTopologicalLayout() {
  static const std::vector<std::size_t> kTreeStructure = {
      ArchetypeTreeStructure::kNoParent, 0, 0
  };
  static const std::vector<ArchetypeJointHandle> kRootParents = {
      ArchetypeJointHandle{0, 0},
      ArchetypeJointHandle{1, 0},
      ArchetypeJointHandle{2, 0}
  };

  std::vector<ArchetypeTreeStructure> tree = {ArchetypeTreeStructure{
      true,
      std::span<const std::size_t>(kTreeStructure),
      std::span<const ArchetypeJointHandle>(kRootParents)
  }};

  return TopologicalOrdering::Build(tree, 2);
}

}  // namespace

TEST(LayoutRows, PaddedSizeIncludesPerJointInstancePaddingNotBaseRow) {
  Layout layout = BuildThreeInstanceTopologicalLayout();

  // 3 joints, each padded to a 4-row block (3 real instances + 1 padding).
  EXPECT_EQ(layout.PaddedSize(), 12U);
  EXPECT_EQ(layout.BaseRowIndex(), 12U);
}

TEST(LayoutRows, ViewInstanceCountRoundsBaseRowUpToLaneMultiple) {
  Layout layout = BuildThreeInstanceTopologicalLayout();

  // BaseRowIndex() + 1 == 13, rounded up to the next multiple of lane_size
  // (2) == 14.
  EXPECT_EQ(layout.ViewInstanceCount(), 14U);
  EXPECT_EQ(layout.LaneSize(), 2U);
}

TEST(LayoutRows, IsPaddingMarksTheUnfilledInstanceSlotOfEveryJointBlock) {
  Layout layout = BuildThreeInstanceTopologicalLayout();

  // Joint 0's block is rows 0-3 (instances 0,1,2 real, row 3 padding);
  // joint 1's is rows 4-7; joint 2's is rows 8-11 -- same shape each time.
  EXPECT_FALSE(layout.IsPadding(0));
  EXPECT_FALSE(layout.IsPadding(1));
  EXPECT_FALSE(layout.IsPadding(2));
  EXPECT_TRUE(layout.IsPadding(3));
  EXPECT_FALSE(layout.IsPadding(4));
  EXPECT_TRUE(layout.IsPadding(7));
  EXPECT_FALSE(layout.IsPadding(8));
  EXPECT_TRUE(layout.IsPadding(11));
}

TEST(LayoutRows, ToSortedAndToPhysicalRoundTrip) {
  Layout layout = BuildThreeInstanceTopologicalLayout();

  // Instance 1, joint 0 (physical index 1*3+0 within the archetype).
  std::size_t row = layout.ToSorted(0, (1 * 3) + 0);
  EXPECT_EQ(row, 1U);
  ArchetypeJointHandle physical = layout.ToPhysical(row);
  EXPECT_EQ(physical.instance_index, 1U);
  EXPECT_EQ(physical.joint_index, 0U);
}

TEST(LayoutTopologyBytes, MatchesPaddedSizeTimesSizeT) {
  Layout layout = BuildThreeInstanceTopologicalLayout();

  EXPECT_EQ(layout.TopologyBytes(), layout.PaddedSize() * sizeof(std::size_t));
}

// AllocateTopology must hand back a JointTopology sized to PaddedSize()
// (real + padding rows only) -- never BaseRowIndex()+1 or
// ViewInstanceCount() -- so the traversal it drives never visits the
// reserved base row as an ordinary target. See Layout::AllocateTopology's
// own comment for why that distinction matters.
TEST(LayoutAllocateTopology, SizeExcludesBaseRowAndTailPadding) {
  Layout layout = BuildThreeInstanceTopologicalLayout();
  Arena arena(layout.TopologyBytes(), alignof(std::size_t));

  auto topology = layout.AllocateTopology(arena);

  EXPECT_EQ(topology.Size(), layout.PaddedSize());
}

// Instance 0's joint 0 (the tree's root) resolves to the reserved base
// row; instance 0's joint 1 (a real child of joint 0) resolves to that
// same instance's own joint-0 row; the padding row at the end of joint 0's
// block defaults to the base row too, per Ordering::BuildFromLocalOrders's
// documented behavior.
TEST(LayoutAllocateTopology, ParentsResolveThroughSameInstancesSortedRows) {
  Layout layout = BuildThreeInstanceTopologicalLayout();
  Arena arena(layout.TopologyBytes(), alignof(std::size_t));
  auto topology = layout.AllocateTopology(arena);

  std::size_t instance0_joint0_row = layout.ToSorted(0, (0 * 3) + 0);
  std::size_t instance0_joint1_row = layout.ToSorted(0, (0 * 3) + 1);

  EXPECT_EQ(topology[instance0_joint0_row], layout.BaseRowIndex());
  EXPECT_EQ(topology[instance0_joint1_row], instance0_joint0_row);
  // Row 3 is joint 0's block's padding row (instance slot 3, unfilled).
  EXPECT_EQ(topology[3], layout.BaseRowIndex());
}

// TopologicalOrdering's depth-sorted placement always leaves every real
// row's parent either strictly earlier or the reserved base row -- exactly
// what IsForwardSafe checks for.
TEST(LayoutIsForwardSafe, TrueForTopologicallySortedLayout) {
  Layout layout = BuildThreeInstanceTopologicalLayout();
  EXPECT_TRUE(layout.IsForwardSafe());
}

// Joint 1 is the tree's actual root and joint 0 is its child, so physical
// order disagrees with dependency order. LinearOrdering's identity
// placement keeps joint 0 (the child) at an earlier row than joint 1 (its
// own parent) -- exactly the shape IsForwardSafe must reject.
TEST(LayoutIsForwardSafe, FalseWhenLinearOrderingContradictsRealDependencies) {
  static const std::vector<std::size_t> kTreeStructure = {
      1, ArchetypeTreeStructure::kNoParent
  };
  static const std::vector<ArchetypeJointHandle> kRootParents = {
      ArchetypeJointHandle{0, 0}
  };
  std::vector<ArchetypeTreeStructure> tree = {ArchetypeTreeStructure{
      true,
      std::span<const std::size_t>(kTreeStructure),
      std::span<const ArchetypeJointHandle>(kRootParents)
  }};

  Layout layout = LinearOrdering::Build(tree, 1);
  EXPECT_FALSE(layout.IsForwardSafe());
}

TEST(LayoutConstructorPrecondition, DiesOnParentsSizeMismatch) {
  std::vector<std::vector<std::size_t>> physical_to_sorted = {{0}};
  std::vector<ArchetypeJointHandle> sorted_to_physical = {
      ArchetypeJointHandle{0, 0}
  };
  // Should be size 2 (sorted_to_physical.size() + 1); deliberately wrong.
  std::vector<std::size_t> parents = {0};

  EXPECT_DEATH(
      Layout(
          std::move(physical_to_sorted),
          std::move(sorted_to_physical),
          std::move(parents),
          1
      ),
      "one more row"
  );
}

TEST(LayoutConstructorPrecondition, DiesOnZeroLaneSize) {
  std::vector<std::vector<std::size_t>> physical_to_sorted = {{0}};
  std::vector<ArchetypeJointHandle> sorted_to_physical = {
      ArchetypeJointHandle{0, 0}
  };
  std::vector<std::size_t> parents = {1, 1};

  EXPECT_DEATH(
      Layout(
          std::move(physical_to_sorted),
          std::move(sorted_to_physical),
          std::move(parents),
          0
      ),
      "lane_size must be positive"
  );
}
