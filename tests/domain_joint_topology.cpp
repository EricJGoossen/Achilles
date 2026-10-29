#include <gtest/gtest.h>

#include <cstddef>
#include <span>
#include <vector>
#include <xsimd/xsimd.hpp>

#include "domain/joint_topology.hpp"

using achilles::domain::JointTopology;

TEST(JointTopologyConstruction, SizeMatchesTheSpanItWasBuiltFrom) {
  std::vector<std::size_t> parents = {
      4, 4, 4, 4
  };  // out of range (== size): no parent

  JointTopology topology{
      std::span<std::size_t>(parents), xsimd::batch<float>::size
  };

  EXPECT_EQ(topology.Size(), 4U);
}

// Every stored value here is out of range (>= size), so CheckBatchSafety
// skips all of them regardless of their relative values -- this isolates
// indexing correctness from batch-safety, which the lane-width-independent
// test below covers separately.
TEST(JointTopologyAccess, IndexingReturnsStoredParents) {
  std::vector<std::size_t> parents = {10, 11, 12, 13};

  JointTopology topology{
      std::span<std::size_t>(parents), xsimd::batch<float>::size
  };

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

  JointTopology topology{std::span<std::size_t>(parents), lane};

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
// IsBatchSafe violation, unlike a relative offset that depends on the
// real xsimd::batch<float>::size to land wrong.
TEST(JointTopologyBatchSafety, SelfParentIsNotBatchSafe) {
  std::vector<std::size_t> parents = {0};

  JointTopology topology{
      std::span<std::size_t>(parents), xsimd::batch<float>::size
  };

  EXPECT_FALSE(topology.IsBatchSafe());
}

TEST(JointTopologyAccessPrecondition, DiesOnOutOfRangeIndex) {
  std::vector<std::size_t> parents = {1};  // out of range: safe by construction

  JointTopology topology{
      std::span<std::size_t>(parents), xsimd::batch<float>::size
  };

  EXPECT_DEATH({ (void)topology[1]; }, "");
}

// ---------------------------------------------------------------------------
// SegmentedTopology / PairTopology
// ---------------------------------------------------------------------------

TEST(SegmentedTopology, ReportsEachSegmentsRows) {
  const std::vector<size_t> parents = {9, 9, 9, 9, 9};
  const std::vector<size_t> segments = {0, 2, 5};
  const achilles::domain::SegmentedTopology topology(
      achilles::domain::JointTopology(parents, 1), segments
  );
  ASSERT_EQ(topology.SegmentCount(), 2U);
  EXPECT_EQ(topology.SegmentBegin(0), 0U);
  EXPECT_EQ(topology.SegmentEnd(0), 2U);
  EXPECT_EQ(topology.SegmentBegin(1), 2U);
  EXPECT_EQ(topology.SegmentEnd(1), 5U);
}

TEST(SegmentedTopology, NoSegmentsIsZeroSegments) {
  const std::vector<size_t> parents;
  const achilles::domain::SegmentedTopology topology(
      achilles::domain::JointTopology(parents, 1), {}
  );
  EXPECT_EQ(topology.SegmentCount(), 0U);
}

TEST(PairTopology, HoldsEachRowsSourceAndPartner) {
  using achilles::domain::PairTopology;
  const std::vector<size_t> sources = {3, 3};
  const std::vector<size_t> partners = {4, PairTopology::kNoPartner};
  const PairTopology topology(
      achilles::domain::JointTopology(sources, 2), partners
  );
  ASSERT_EQ(topology.Size(), 2U);
  EXPECT_EQ(topology[0], 3U);
  EXPECT_EQ(topology[1], 3U);
  EXPECT_EQ(topology.Partner(0), 4U);
  EXPECT_EQ(topology.Partner(1), PairTopology::kNoPartner);
}
