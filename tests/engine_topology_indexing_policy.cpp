#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <tuple>
#include <type_traits>
#include <vector>

#include "domain/joint_layout.hpp"
#include "engine/topology/indexing_policy.hpp"

// Covers topology::GlobalIndexing: one element for the whole field, read
// plainly by a scalar load and as a BroadcastIndex by a batched one -- the
// split that decides, at compile time, whether an argument is a whole-sim
// value a GlobalTraversal Op updates or a reduction a batched pass adds
// into.

using achilles::engine::topology::BroadcastIndex;
using achilles::engine::topology::GlobalIndexing;
using achilles::engine::topology::IndexingPolicyLike;

static_assert(IndexingPolicyLike<GlobalIndexing>);

namespace {

// A tree pass's (target group, parent group) and a pair pass's (target
// group, source row) -- GlobalIndexing must ignore both shapes alike.
const std::tuple<std::size_t, std::size_t> kTreeIndices{3, 1};
const std::tuple<std::size_t, BroadcastIndex> kPairIndices{
    2, BroadcastIndex{5}
};

template <std::size_t LoadLanes>
void ExpectBatchedResolvesToBroadcastZero() {
  using Resolved = decltype(GlobalIndexing::Resolve<0, LoadLanes>(
      kTreeIndices, std::size_t{0}
  ));
  static_assert(std::is_same_v<Resolved, BroadcastIndex>);

  EXPECT_EQ((GlobalIndexing::Resolve<0, LoadLanes>(kTreeIndices, 64).row), 0U);
  EXPECT_EQ((GlobalIndexing::Resolve<1, LoadLanes>(kTreeIndices, 64).row), 0U);
  EXPECT_EQ((GlobalIndexing::Resolve<1, LoadLanes>(kPairIndices, 64).row), 0U);
}

}  // namespace

// One element whatever the row count -- including none, so a sim's global
// scalars exist (and are carved) even before it has any rows.
TEST(GlobalIndexingCount, IsOneForAnyRowCount) {
  EXPECT_EQ(GlobalIndexing::Count(0), 1U);
  EXPECT_EQ(GlobalIndexing::Count(1), 1U);
  EXPECT_EQ(GlobalIndexing::Count(1000), 1U);
}

// A scalar load reads/writes the one element plainly (a plain index, not a
// BroadcastIndex), whichever slot and indices the traversal handed --
// what a GlobalTraversal Op updates the value through.
TEST(GlobalIndexingResolve, ScalarLoadResolvesToPlainIndexZero) {
  using Resolved =
      decltype(GlobalIndexing::Resolve<0, 1>(kTreeIndices, std::size_t{0}));
  static_assert(std::is_same_v<Resolved, std::size_t>);

  EXPECT_EQ((GlobalIndexing::Resolve<0, 1>(kTreeIndices, 64)), 0U);
  EXPECT_EQ((GlobalIndexing::Resolve<1, 1>(kTreeIndices, 64)), 0U);
  EXPECT_EQ((GlobalIndexing::Resolve<0, 1>(kPairIndices, 64)), 0U);
}

// A batched load resolves to BroadcastIndex{0} -- splatted as an input, a
// lane reduction as an output -- at every lane width (TESTING.md 9.4).
TEST(GlobalIndexingResolve, BatchedLoadResolvesToBroadcastZeroAtFourLanes) {
  ExpectBatchedResolvesToBroadcastZero<4>();
}

TEST(GlobalIndexingResolve, BatchedLoadResolvesToBroadcastZeroAtEightLanes) {
  ExpectBatchedResolvesToBroadcastZero<8>();
}

TEST(
    GlobalIndexingResolve, BatchedLoadResolvesToBroadcastZeroAtThirtyTwoLanes
) {
  ExpectBatchedResolvesToBroadcastZero<32>();
}

// The one element is always real, so nothing is ever padding-seeded --
// seeding it would clobber a value a pass carries across ticks (e.g. the
// solver's line-search state).
TEST(GlobalIndexingSeedPadding, WritesNothing) {
  std::vector<std::size_t> written;
  // Empty, but well-formed: no instances, no rows.
  const std::array<std::size_t, 1> instance_offsets = {0};
  achilles::domain::JointLayout layout(instance_offsets, {}, {});
  GlobalIndexing::SeedPadding(
      [&](std::size_t element) { written.push_back(element); }, layout, 16
  );
  EXPECT_TRUE(written.empty());
}
