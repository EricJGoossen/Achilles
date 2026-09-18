#include <gtest/gtest.h>

#include <cstddef>
#include <span>
#include <vector>
#include <xsimd/xsimd.hpp>

#include "domain/joint_topology.hpp"

using achilles::domain::JointTopology;

TEST(JointTopologyConstruction, SizeMatchesTheSpanItWasBuiltFrom) {
  std::vector<std::size_t> parents = {4, 4, 4, 4};  // out of range (== size): no parent

  JointTopology topology{std::span<std::size_t>(parents)};

  EXPECT_EQ(topology.Size(), 4U);
}

// Every stored value here is out of range (>= size), so CheckBatchSafety
// skips all of them regardless of their relative values -- this isolates
// indexing correctness from batch-safety, which the lane-width-independent
// test below covers separately.
TEST(JointTopologyAccess, IndexingReturnsStoredParents) {
  std::vector<std::size_t> parents = {10, 11, 12, 13};

  JointTopology topology{std::span<std::size_t>(parents)};

  EXPECT_EQ(topology[0], 10U);
  EXPECT_EQ(topology[1], 11U);
  EXPECT_EQ(topology[2], 12U);
  EXPECT_EQ(topology[3], 13U);
}

// A lane-width-independent two-level tree: the first `lane` rows are roots
// (their stored parent is `2*lane`, i.e. Size() -- out of range, the "no
// parent" convention CheckBatchSafety's `parent >= size` skip relies on),
// and the second `lane` rows each point back at the root `lane` positions
// earlier. Root and child always land in different lane groups --
// (root's index)/lane == 0, (child's index)/lane == 1 -- regardless of
// what xsimd::batch<float>::size actually is on this build.
TEST(JointTopologyConstruction, AcceptsATwoLevelTreePaddedToTheRealLaneWidth) {
  const std::size_t lane = xsimd::batch<float>::size;
  const std::size_t n = 2 * lane;

  std::vector<std::size_t> parents(n);
  for (std::size_t i = 0; i < lane; ++i) {
    parents[i] = n;  // root: parent out of range, no substitution needed.
  }
  for (std::size_t i = lane; i < n; ++i) {
    parents[i] = i - lane;
  }

  JointTopology topology{std::span<std::size_t>(parents)};

  EXPECT_EQ(topology.Size(), n);
  for (std::size_t i = 0; i < lane; ++i) {
    EXPECT_EQ(topology[i], n);
  }
  for (std::size_t i = lane; i < n; ++i) {
    EXPECT_EQ(topology[i], i - lane);
  }
}

// A stored parent index equal to its own position always lands in the same
// lane group as itself, for any lane width -- a portable, guaranteed
// CheckBatchSafety violation, unlike a relative offset that depends on the
// real xsimd::batch<float>::size to land wrong.
TEST(JointTopologyConstructorPrecondition, DiesOnSelfParentBatchUnsafety) {
  std::vector<std::size_t> parents = {0};

  EXPECT_DEATH(
      JointTopology(std::span<std::size_t>(parents)), "same SIMD batch"
  );
}

TEST(JointTopologyAccessPrecondition, DiesOnOutOfRangeIndex) {
  std::vector<std::size_t> parents = {1};  // out of range: safe by construction

  JointTopology topology{std::span<std::size_t>(parents)};

  EXPECT_DEATH({ (void)topology[1]; }, "");
}
