#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "domain/archetype.hpp"
#include "domain/joint_layout.hpp"
#include "domain/joint_topology.hpp"
#include "engine/pass/row_policies.hpp"
#include "engine/pass/traversals.hpp"
#include "engine/topology/indexing_policy.hpp"

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
// engine_pass_algorithm_step.cpp for that pairing exercised for real).
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
// See engine_pass_algorithm_step.cpp for that pairing exercised for real.
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
// raw rows (JointLayout::PaddedSize()), but OpInvoker feeds a traversal's index
// straight into a batched View::Load/Store, which addresses storage in
// units of Stride-sized groups (see TreeTraversal's own comment). Every
// test above uses ForwardTreeTraversal/ForwardLinearTraversal (and their
// Backward counterparts), which hardcode Stride=1 -- there's no default to
// fall back on (Stride is a mandatory second template argument on
// TreeTraversal/LinearTraversal themselves) -- where group == raw row --
// these exercise Stride > 1 directly against TreeTraversal/LinearTraversal
// in isolation, independent of any real View/SimAllocator, at every lane
// size TESTING.md 9.4 requires (4, 8, 32).
// ---------------------------------------------------------------------------

static_assert(
    achilles::engine::pass::TreeTraversal<Direction::kForward, 1>::kStride == 1
);
static_assert(
    achilles::engine::pass::LinearTraversal<Direction::kForward, 1>::kStride ==
    1
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

// ---------------------------------------------------------------------------
// GlobalTraversal: one call at index 0, for an Op over GlobalIndexing
// fields. It takes whatever the Step call's other traversals take and
// ignores it, so it can sit in the same Ops list as tree/linear/pair passes.
// Always Stride 1 -- there's only the one element -- so TESTING.md 9.4's
// lane sizes don't apply to it.
// ---------------------------------------------------------------------------

namespace {

using achilles::engine::pass::GlobalTraversal;

static_assert(TraversalLike<GlobalTraversal>);
static_assert(GlobalTraversal::kStride == 1);
static_assert(GlobalTraversal::kDirection == Direction::kForward);

}  // namespace

// Exactly one call, (0, 0) -- whatever the topology it's handed holds,
// including nothing at all. A sim's global scalars still need updating on a
// tick with no rows to reduce over.
TEST(GlobalTraversalApply, CallsOnceAtZeroWhateverTheTopology) {
  for (std::size_t size : {0U, 1U, 7U}) {
    std::vector<std::pair<size_t, size_t>> visits;
    TopologyArchetype topology{std::vector<size_t>(size, 0)};
    GlobalTraversal::Apply(
        [&](size_t target, size_t source) { visits.emplace_back(target, source); },
        topology
    );
    ASSERT_EQ(visits.size(), 1U);
    EXPECT_EQ(visits[0].first, 0U);
    EXPECT_EQ(visits[0].second, 0U);
  }
}

// Any further arguments (a Step call's topology, or none) are ignored.
TEST(GlobalTraversalApply, AcceptsNoTopologyArgument) {
  std::size_t calls = 0;
  GlobalTraversal::Apply([&](size_t, size_t) { ++calls; });
  EXPECT_EQ(calls, 1U);
}

// Initialize runs at that same index 0, unconditionally -- unlike the
// tree/linear traversals, which skip it for an empty topology, since the
// one global element always exists.
TEST(GlobalTraversalInitOp, AlwaysInitializesAtZero) {
  size_t seen = 999;
  GlobalTraversal::InitOp(RecordingInitOp{&seen}, TopologyArchetype{{}});
  EXPECT_EQ(seen, 0U);
}

// ---------------------------------------------------------------------------
// PairTraversal
// ---------------------------------------------------------------------------

namespace {

using achilles::domain::JointTopology;
using achilles::domain::PairTopology;
using achilles::engine::pass::PairStructure;
using achilles::engine::pass::PairTraversal;
using achilles::engine::topology::BroadcastIndex;

struct PairCall {
  size_t group;
  size_t source;
  size_t partner_group;
  std::uint64_t lanes;
};

struct RecordingPairs {
  std::vector<PairCall>* calls;
  void operator()(
      size_t group,
      BroadcastIndex source,
      size_t partner_group,
      PairStructure structure
  ) const {
    calls->push_back(
        {group, source.row, partner_group, structure.distinct_lanes}
    );
  }
};

// Two lane groups of four pairs: source row 1 against body rows 4..7 (lane
// 2 padding), then source row 5 against body rows 8..11 (only lane 0).
struct TwoGroups {
  static constexpr size_t kNo = PairTopology::kNoPartner;
  std::vector<size_t> sources = {1, 1, 1, 1, 5, 5, 5, 5};
  std::vector<size_t> partners = {4, 5, kNo, 7, 8, kNo, kNo, kNo};
  PairTopology Topology() const {
    return {JointTopology(sources, 4), partners};
  }
};

}  // namespace

static_assert(
    TraversalLike<PairTraversal<Direction::kForward, 4>>,
    "PairTraversal must satisfy TraversalLike concept"
);

TEST(PairTraversalApply, VisitsEachLaneGroupWithItsSourcePartnersAndLanes) {
  TwoGroups pairs;
  std::vector<PairCall> calls;
  PairTraversal<Direction::kForward, 4>::Apply(
      RecordingPairs{&calls}, size_t{99}, pairs.Topology()
  );
  ASSERT_EQ(calls.size(), 2U);
  EXPECT_EQ(calls[0].group, 0U);
  EXPECT_EQ(calls[0].source, 1U);
  EXPECT_EQ(calls[0].partner_group, 1U);
  EXPECT_EQ(calls[0].lanes, 0b1011U);
  EXPECT_EQ(calls[1].group, 1U);
  EXPECT_EQ(calls[1].source, 5U);
  EXPECT_EQ(calls[1].partner_group, 2U);
  EXPECT_EQ(calls[1].lanes, 0b0001U);
}

TEST(PairTraversalApply, BackwardVisitsTheGroupsInReverse) {
  TwoGroups pairs;
  std::vector<PairCall> calls;
  PairTraversal<Direction::kBackward, 4>::Apply(
      RecordingPairs{&calls}, size_t{99}, pairs.Topology()
  );
  ASSERT_EQ(calls.size(), 2U);
  EXPECT_EQ(calls[0].group, 1U);
  EXPECT_EQ(calls[1].group, 0U);
}

TEST(PairTraversalApply, NoPairsMakesNoCalls) {
  const std::vector<size_t> none;
  std::vector<PairCall> calls;
  PairTraversal<Direction::kForward, 4>::Apply(
      RecordingPairs{&calls},
      size_t{99},
      PairTopology(JointTopology(none, 4), none)
  );
  EXPECT_TRUE(calls.empty());
}

// A pass over pairs that reduces into a whole-sim value starts it from its
// Initialize, pairs or not -- passes over bodies may add to it after.
TEST(PairTraversalInitOp, InitializesEvenWithNoPairs) {
  const std::vector<size_t> none;
  size_t seen = 999;
  PairTraversal<Direction::kForward, 4>::InitOp(
      RecordingInitOp{&seen},
      size_t{99},
      PairTopology(JointTopology(none, 4), none)
  );
  EXPECT_EQ(seen, 0U);
}

// ---------------------------------------------------------------------------
// Linked, segment and scanning passes over a list ordering
// ---------------------------------------------------------------------------

namespace {

using achilles::domain::ArchetypeJointHandle;
using achilles::domain::JointLayout;
using achilles::domain::SegmentedTopology;
using achilles::engine::pass::LinkedSegmentScanTraversal;
using achilles::engine::pass::LinkedTraversal;
using achilles::engine::pass::SegmentTraversal;
using achilles::engine::pass::SelfSegmentTraversal;

struct RecordingRows {
  std::vector<std::pair<size_t, size_t>>* calls;
  void operator()(size_t target, size_t source) const {
    calls->emplace_back(target, source);
  }
};

struct ScanCall {
  size_t target;
  size_t source;
  size_t scene;
};

struct RecordingScan {
  std::vector<ScanCall>* calls;
  void operator()(size_t target, size_t source, size_t scene) const {
    calls->push_back({target, source, scene});
  }
};

// Five list rows in two segments, [0, 2) and [2, 5), each linked to a row
// of some other ordering.
struct TwoLists {
  std::vector<size_t> links = {10, 11, 12, 13, 14};
  std::vector<size_t> segments = {0, 2, 5};
  SegmentedTopology Topology() const {
    return {JointTopology(links, 1), segments};
  }
};

// Three rows of a scene, the middle one padding.
struct Scene {
  static constexpr size_t kNo = JointLayout::kNoInstance;
  std::vector<size_t> offsets = {0, 2};
  std::vector<size_t> sorted = {0, 2};
  std::vector<ArchetypeJointHandle> joints = {{0, 0}, {kNo, kNo}, {0, 1}};
  JointLayout Layout() const { return {offsets, sorted, joints}; }
};

}  // namespace

TEST(LinkedTraversalApply, HandsEachRowItsLink) {
  TwoLists lists;
  std::vector<std::pair<size_t, size_t>> calls;
  LinkedTraversal::Apply(RecordingRows{&calls}, lists.Topology());
  ASSERT_EQ(calls.size(), 5U);
  EXPECT_EQ(calls[3], (std::pair<size_t, size_t>{3, 13}));
}

TEST(SegmentTraversalApply, VisitsOnlyItsSegmentsRows) {
  TwoLists lists;
  std::vector<std::pair<size_t, size_t>> linked;
  SegmentTraversal<1>::Apply(RecordingRows{&linked}, lists.Topology());
  ASSERT_EQ(linked.size(), 3U);
  EXPECT_EQ(linked.front(), (std::pair<size_t, size_t>{2, 12}));
  EXPECT_EQ(linked.back(), (std::pair<size_t, size_t>{4, 14}));

  std::vector<std::pair<size_t, size_t>> self;
  SelfSegmentTraversal<0>::Apply(RecordingRows{&self}, lists.Topology());
  ASSERT_EQ(self.size(), 2U);
  EXPECT_EQ(self.back(), (std::pair<size_t, size_t>{1, 1}));
}

TEST(LinkedSegmentScanTraversalApply, PairsEachRowWithEveryRealSceneRow) {
  TwoLists lists;
  Scene scene;
  std::vector<ScanCall> calls;
  LinkedSegmentScanTraversal<0>::Apply(
      RecordingScan{&calls}, lists.Topology(), scene.Layout()
  );
  // Two rows times the two real scene rows; the padding row is skipped.
  ASSERT_EQ(calls.size(), 4U);
  EXPECT_EQ(calls[0].scene, 0U);
  EXPECT_EQ(calls[1].scene, 2U);
  EXPECT_EQ(calls[2].target, 1U);
  EXPECT_EQ(calls[2].source, 11U);
}

TEST(LinkedSegmentScanTraversalInitOp, SkipsInitializeWithNoSceneToScan) {
  TwoLists lists;
  size_t seen = 999;
  LinkedSegmentScanTraversal<0>::InitOp(
      RecordingInitOp{&seen}, lists.Topology()
  );
  EXPECT_EQ(seen, 999U);
}
