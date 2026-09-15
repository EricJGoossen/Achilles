#include <gtest/gtest.h>

#include <cstddef>
#include <vector>
#include <xsimd/xsimd.hpp>

#include "domain/topology/joint_topology.hpp"
#include "support/joint_topology_fixture.hpp"

using achilles::domain::topology::SegmentData;
using achilles::test_support::TopologyFixture;

namespace {

// Two-level, lane-width-independent tree: the first `lane` joints are
// roots (their stored parent is `2*lane`, i.e. Size() -- out of range, the
// "no parent" convention CheckBatchSafety's `parent >= size` skip relies
// on), and the second `lane` joints each point back at the root `lane`
// positions earlier. Root and child always land in different
// lane-group -- (root's index)/lane == 0, (child's index)/lane == 1 --
// regardless of what xsimd::batch<float>::size actually is on this build,
// so this is safe to construct at TraversalLike's/CheckBatchSafety's
// strictest on any target.
TopologyFixture TwoLevelTopology() {
  const size_t lane = xsimd::batch<float>::size;
  const size_t n = 2 * lane;

  std::vector<size_t> flat(n);
  for (size_t i = 0; i < lane; ++i) {
    flat[i] = n;  // root: parent out of range, no substitution needed.
  }
  for (size_t i = lane; i < n; ++i) {
    flat[i] = i - lane;
  }

  return TopologyFixture({flat}, {SegmentData{0, 0}});
}

}  // namespace

TEST(JointTopologyConstruction, SizeMatchesSegmentTotal) {
  TopologyFixture fixture = TwoLevelTopology();
  EXPECT_EQ(fixture.topology.Size(), 2 * xsimd::batch<float>::size);
}

TEST(JointTopologyAccess, IndexingReturnsStoredParents) {
  TopologyFixture fixture = TwoLevelTopology();
  const size_t lane = xsimd::batch<float>::size;
  const size_t n = 2 * lane;

  for (size_t i = 0; i < lane; ++i) {
    EXPECT_EQ(fixture.topology[i], n);  // root: out-of-range "no parent".
  }
  for (size_t i = lane; i < n; ++i) {
    EXPECT_EQ(fixture.topology[i], i - lane);
  }
}

// A JointTopology assembled from more than one segment sums each
// segment's contribution to Size() and keeps each segment's own indices
// distinct -- there's no requirement that segments be the same size.
// Reuses TwoLevelTopology's lane-width-independent root/child split
// (see there), just spread across two segments instead of one, so this
// stays batch-safe on any build regardless of xsimd::batch<float>::size.
TEST(JointTopologyAccess, MultipleSegmentsConcatenate) {
  const size_t lane = xsimd::batch<float>::size;
  const size_t n = 2 * lane;

  std::vector<size_t> roots(lane, n);  // out of range: no parent.
  std::vector<size_t> children(lane);
  for (size_t k = 0; k < lane; ++k) {
    children[k] = k;  // global index of the corresponding root.
  }

  TopologyFixture fixture(
      {roots, children}, {SegmentData{0, 0}, SegmentData{1, 0}}
  );

  EXPECT_EQ(fixture.topology.Size(), n);
  for (size_t i = 0; i < lane; ++i) {
    EXPECT_EQ(fixture.topology[i], n);
  }
  for (size_t i = lane; i < n; ++i) {
    EXPECT_EQ(fixture.topology[i], i - lane);
  }
}

// kInterSegmentParent in a segment's data is a sentinel, not a literal
// parent index: operator[] substitutes that segment's own parent_joint in
// its place instead of returning the sentinel value itself.
TEST(JointTopologyAccess, InterSegmentParentSubstitutesSegmentParentJoint) {
  size_t inter = achilles::domain::topology::kInterSegmentParent;
  TopologyFixture fixture({{inter}}, {SegmentData{0, 7}});
  EXPECT_EQ(fixture.topology[0], 7U);
}

TEST(JointTopologyValidation, RejectsSegmentIndexOutOfRange) {
  // segment_data names segment 1, but only segment 0 exists.
  EXPECT_DEATH(
      TopologyFixture({{0}}, {SegmentData{1, 0}}), "Inconsistent data"
  );
}

// A stored parent index equal to its own position (in-range, not the
// out-of-range "no parent" convention used above) always lands in the
// same lane group as itself, for any lane width -- a portable, guaranteed
// CheckBatchSafety violation.
TEST(JointTopologyValidation, RejectsSelfParentBatchUnsafety) {
  EXPECT_DEATH(TopologyFixture({{0}}, {SegmentData{0, 0}}), "batch");
}

TEST(JointTopologyValidation, RejectsOutOfRangeIndex) {
  TopologyFixture fixture = TwoLevelTopology();
  EXPECT_DEATH(
      { (void)fixture.topology[fixture.topology.Size()]; }, "Index out of range"
  );
}
