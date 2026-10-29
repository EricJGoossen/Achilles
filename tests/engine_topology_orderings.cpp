#include <gtest/gtest.h>

#include <cstddef>
#include <span>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>
#include <xsimd/xsimd.hpp>

#include "domain/archetype.hpp"
#include "domain/joint_layout.hpp"
#include "domain/joint_topology.hpp"
#include "engine/memory/arena.hpp"
#include "engine/topology/ordering_plan.hpp"
#include "engine/topology/orderings.hpp"
#include "util/tmp.hpp"

using achilles::domain::ArchetypeJointHandle;
using achilles::domain::ArchetypeTreeStructure;
using achilles::domain::JointLayout;
using achilles::domain::JointTopology;
using achilles::engine::memory::Arena;
using achilles::engine::topology::LinearOrdering;
using achilles::engine::topology::OrderingPlan;
using achilles::engine::topology::OrderingPolicyLike;
using achilles::engine::topology::TopologicalOrdering;

namespace {

constexpr std::size_t kNoParent = ArchetypeTreeStructure::kNoParent;

ArchetypeTreeStructure MakeArchetype(
    bool is_root,
    const std::vector<std::size_t>& tree_structure,
    const std::vector<ArchetypeJointHandle>& root_parents
) {
  return ArchetypeTreeStructure{
      is_root,
      std::span<const std::size_t>(tree_structure),
      std::span<const ArchetypeJointHandle>(root_parents)
  };
}

// Exactly what OrderingPolicyLike requires. Nothing else.
struct OrderingPolicyArchetype {
  static OrderingPlan Build(
      std::span<ArchetypeTreeStructure> tree, std::size_t lane_size
  );
};
static_assert(OrderingPolicyLike<OrderingPolicyArchetype>);

}  // namespace

static_assert(OrderingPolicyLike<TopologicalOrdering>);
static_assert(OrderingPolicyLike<LinearOrdering>);

// ---------------------------------------------------------------------------
// TopologicalOrdering::Build -- single archetype
// ---------------------------------------------------------------------------

// The root archetype's instances have no external parent -- Consolidate
// reports kNoParent for them, and BuildFromLocalOrders substitutes the
// reserved base row.
TEST(TopologicalOrderingSingleArchetype, RootInstancesResolveToBaseRow) {
  std::vector<std::size_t> tree_structure = {kNoParent};
  std::vector<ArchetypeJointHandle> root_parents(
      3, ArchetypeJointHandle{kNoParent, kNoParent}
  );
  std::vector<ArchetypeTreeStructure> tree = {
      MakeArchetype(true, tree_structure, root_parents)
  };

  OrderingPlan plan = TopologicalOrdering::Build(tree, 4);
  JointLayout layout = plan.Layout();
  JointTopology topology = plan.Topology();

  for (std::size_t i = 0; i < 3; ++i) {
    std::size_t row = layout.ToSorted({i, 0});
    EXPECT_EQ(topology[row], layout.BaseRowIndex());
  }
  // 3 instances padded to a lane multiple of 4 leaves one padding slot,
  // defaulted to the base row the same way a real root instance is.
  EXPECT_TRUE(layout.IsPadding(3));
  EXPECT_EQ(topology[3], layout.BaseRowIndex());
}

// A straight 3-joint chain within one instance, with the raw array
// deliberately NOT already in dependency order (joint 0's parent is
// joint 2, joint 2's parent is joint 1, joint 1 is the root) -- proving
// SortTree finds the root wherever it sits and linearizes the true chain
// (joint1 -> joint2 -> joint0), not the raw index order.
TEST(TopologicalOrderingSingleArchetype, LinearizesAScrambledChain) {
  std::vector<std::size_t> tree_structure = {2, kNoParent, 1};
  std::vector<ArchetypeJointHandle> root_parents = {
      ArchetypeJointHandle{kNoParent, kNoParent}
  };
  std::vector<ArchetypeTreeStructure> tree = {
      MakeArchetype(true, tree_structure, root_parents)
  };

  OrderingPlan plan = TopologicalOrdering::Build(tree, 1);
  JointLayout layout = plan.Layout();

  EXPECT_EQ(layout.ToSorted({0, 1}), 0U);  // joint1: the root, sorted first
  EXPECT_EQ(layout.ToSorted({0, 2}), 1U);  // joint2: joint1's child
  EXPECT_EQ(layout.ToSorted({0, 0}), 2U);  // joint0: joint2's child
  EXPECT_TRUE(plan.Topology().IsForwardSafe());
}

// A branching tree (joint 0's two children are joints 1 and 2) --
// SortTree's BFS visits a level's nodes in ascending physical-index order,
// so siblings get a deterministic (not just "any valid") relative order.
TEST(
    TopologicalOrderingSingleArchetype,
    BranchingTreeOrdersSiblingsByPhysicalIndex
) {
  std::vector<std::size_t> tree_structure = {kNoParent, 0, 0};
  std::vector<ArchetypeJointHandle> root_parents = {
      ArchetypeJointHandle{kNoParent, kNoParent}
  };
  std::vector<ArchetypeTreeStructure> tree = {
      MakeArchetype(true, tree_structure, root_parents)
  };

  OrderingPlan plan = TopologicalOrdering::Build(tree, 1);
  JointLayout layout = plan.Layout();

  EXPECT_EQ(layout.ToSorted({0, 0}), 0U);
  EXPECT_EQ(layout.ToSorted({0, 1}), 1U);
  EXPECT_EQ(layout.ToSorted({0, 2}), 2U);
  EXPECT_TRUE(plan.Topology().IsForwardSafe());
}

// 3 instances against lane_size 2: each of the archetype's 2 joint
// positions gets its own RoundUp(3, 2) == 4-row block (3 real, 1 padding),
// so PaddedSize() == 8, not 6 -- the padding lives per joint position, not
// once at the end.
TEST(TopologicalOrderingSingleArchetype, PadsEveryJointPositionToLaneMultiple) {
  std::vector<std::size_t> tree_structure = {kNoParent, 0};
  std::vector<ArchetypeJointHandle> root_parents = {
      ArchetypeJointHandle{kNoParent, kNoParent},
      ArchetypeJointHandle{kNoParent, kNoParent},
      ArchetypeJointHandle{kNoParent, kNoParent}
  };
  std::vector<ArchetypeTreeStructure> tree = {
      MakeArchetype(true, tree_structure, root_parents)
  };

  OrderingPlan plan = TopologicalOrdering::Build(tree, 2);
  JointLayout layout = plan.Layout();

  ASSERT_EQ(layout.PaddedSize(), 8U);
  EXPECT_FALSE(layout.IsPadding(0));
  EXPECT_FALSE(layout.IsPadding(1));
  EXPECT_FALSE(layout.IsPadding(2));
  EXPECT_TRUE(layout.IsPadding(3));
  EXPECT_FALSE(layout.IsPadding(4));
  EXPECT_FALSE(layout.IsPadding(5));
  EXPECT_FALSE(layout.IsPadding(6));
  EXPECT_TRUE(layout.IsPadding(7));
  EXPECT_EQ(layout.ToPhysical(3).instance_index, kNoParent);
}

// Same shape as PadsEveryJointPositionToLaneMultiple above, but at a much
// wider lane size (32, e.g. AVX-512 float lanes) than this build's actual
// native xsimd::batch<float>::size -- Consolidate's padding math (RoundUp)
// is generic over lane_size, and TESTING.md 9.4 requires lane sizes of (at
// least) 4, 8, and 32 be exercised, not just whatever's native here.
TEST(TopologicalOrderingSingleArchetype, PadsToLaneMultipleOfThirtyTwo) {
  std::vector<std::size_t> tree_structure = {kNoParent, 0};
  std::vector<ArchetypeJointHandle> root_parents(
      5, ArchetypeJointHandle{kNoParent, kNoParent}
  );
  std::vector<ArchetypeTreeStructure> tree = {
      MakeArchetype(true, tree_structure, root_parents)
  };

  OrderingPlan plan = TopologicalOrdering::Build(tree, 32);
  JointLayout layout = plan.Layout();

  // 2 joint positions, 5 real instances each padded up to 32 -> 64 rows.
  ASSERT_EQ(layout.PaddedSize(), 64U);
  for (std::size_t position = 0; position < 2; ++position) {
    std::size_t base = position * 32;
    for (std::size_t i = 0; i < 5; ++i) {
      EXPECT_FALSE(layout.IsPadding(base + i)) << "row " << (base + i);
    }
    for (std::size_t i = 5; i < 32; ++i) {
      EXPECT_TRUE(layout.IsPadding(base + i)) << "row " << (base + i);
    }
  }
}

// ---------------------------------------------------------------------------
// TopologicalOrdering::Build -- multiple archetypes
// ---------------------------------------------------------------------------

// Two archetypes: archetype 0 is the root; archetype 1 depends on it.
// Each instance of archetype 1 attaches to a *different* instance of
// archetype 0, proving root_parents is resolved per instance, not just
// per archetype.
TEST(TopologicalOrderingMultipleArchetypes, EachInstanceResolvesItsOwnTarget) {
  std::vector<std::size_t> root_tree = {kNoParent};
  std::vector<ArchetypeJointHandle> root_roots(
      2, ArchetypeJointHandle{kNoParent, kNoParent}
  );

  std::vector<std::size_t> dependent_tree = {kNoParent};
  // Global instance numbering: archetype 0's instances are 0 and 1.
  std::vector<ArchetypeJointHandle> dependent_roots = {
      ArchetypeJointHandle{0, 0}, ArchetypeJointHandle{1, 0}
  };

  std::vector<ArchetypeTreeStructure> tree = {
      MakeArchetype(true, root_tree, root_roots),
      MakeArchetype(false, dependent_tree, dependent_roots)
  };

  // lane_size must be a real SIMD batch width here, not an arbitrary small
  // number: plan.Topology() below builds a real JointTopology, whose own
  // constructor checks batch safety against the *actual*
  // xsimd::batch<float>::size, not whatever lane_size this Build call was
  // given. Each archetype's block is only lane_size rows wide (2 instances,
  // 1 joint), so a lane_size smaller than the real batch width would let
  // the root's block and the dependent's block share one real batch --
  // exactly the cross-archetype case being tested here.
  const std::size_t lane_size = xsimd::batch<float>::size;
  OrderingPlan plan = TopologicalOrdering::Build(tree, lane_size);
  JointLayout layout = plan.Layout();
  JointTopology topology = plan.Topology();

  // Global instances: the root's are 0 and 1, the dependent's 2 and 3.
  EXPECT_EQ(topology[layout.ToSorted({2, 0})], layout.ToSorted({0, 0}));
  EXPECT_EQ(topology[layout.ToSorted({3, 0})], layout.ToSorted({1, 0}));
}

// The dependent archetype (index 0 in `tree`) is placed AFTER the root
// archetype (index 1 in `tree`) despite appearing first -- placement order
// follows dependency depth, not `tree`'s own order.
TEST(
    TopologicalOrderingMultipleArchetypes, PlacesShallowerDepthArchetypesFirst
) {
  std::vector<std::size_t> dependent_tree = {kNoParent};
  std::vector<ArchetypeJointHandle> dependent_roots = {
      ArchetypeJointHandle{
          1, 0
      }  // global instance 1: the root archetype's only instance
  };

  std::vector<std::size_t> root_tree = {kNoParent};
  std::vector<ArchetypeJointHandle> root_roots = {
      ArchetypeJointHandle{kNoParent, kNoParent}
  };

  std::vector<ArchetypeTreeStructure> tree = {
      MakeArchetype(false, dependent_tree, dependent_roots),
      MakeArchetype(true, root_tree, root_roots)
  };

  OrderingPlan plan = TopologicalOrdering::Build(tree, 1);
  JointLayout layout = plan.Layout();

  EXPECT_LT(layout.ToSorted({1, 0}), layout.ToSorted({0, 0}));
  EXPECT_TRUE(plan.Topology().IsForwardSafe());
}

// A 3-archetype chain (root -> mid -> leaf) with non-lane-aligned instance
// counts (5, 6, 10 against lane_size 8) and scattered, non-parallel
// cross-archetype attachments -- close to a real hosted-archetype shape,
// and specifically the case batch-safety (see JointTopology::
// CheckBatchSafety, asserted inside plan.Topology() below) has to hold
// for regardless of how the archetypes' block sizes interact.
TEST(
    TopologicalOrderingMultipleArchetypes,
    ThreeArchetypeChainIsBatchSafeAndCorrect
) {
  std::vector<std::size_t> root_tree = {kNoParent};
  std::vector<ArchetypeJointHandle> root_roots(
      5, ArchetypeJointHandle{kNoParent, kNoParent}
  );

  std::vector<std::size_t> mid_tree = {kNoParent, 0, 1};
  std::vector<ArchetypeJointHandle> mid_roots = {
      ArchetypeJointHandle{0, 0},
      ArchetypeJointHandle{1, 0},
      ArchetypeJointHandle{2, 0},
      ArchetypeJointHandle{3, 0},
      ArchetypeJointHandle{4, 0},
      ArchetypeJointHandle{0, 0}
  };

  std::vector<std::size_t> leaf_tree = {kNoParent, 0};
  std::vector<ArchetypeJointHandle> leaf_roots;
  for (std::size_t i = 0; i < 10; ++i) {
    // Global instance numbering: mid's instances start at 5.
    leaf_roots.push_back(ArchetypeJointHandle{5 + (i * 3) % 6, i % 3});
  }

  std::vector<ArchetypeTreeStructure> tree = {
      MakeArchetype(true, root_tree, root_roots),
      MakeArchetype(false, mid_tree, mid_roots),
      MakeArchetype(false, leaf_tree, leaf_roots)
  };

  OrderingPlan plan = TopologicalOrdering::Build(tree, 8);
  JointLayout layout = plan.Layout();
  // No parent/child pair landed in the same SIMD batch.
  JointTopology topology = plan.Topology();
  EXPECT_TRUE(topology.IsBatchSafe());

  EXPECT_TRUE(plan.Topology().IsForwardSafe());

  // Spot-check a couple of resolved cross-archetype rows against the hand-
  // computed instance mapping above. Global instances: root 0-4, mid 5-10,
  // leaf 11-20.
  EXPECT_EQ(
      topology[layout.ToSorted({5, 0})], layout.ToSorted({0, 0})
  );  // mid instance0 -> root instance0
  EXPECT_EQ(
      topology[layout.ToSorted({6, 0})], layout.ToSorted({1, 0})
  );  // mid instance1 -> root instance1
  // leaf instance0's target global instance is 5 + (0*3)%6 == 5: mid's
  // first instance.
  EXPECT_EQ(topology[layout.ToSorted({11, 0})], layout.ToSorted({5, 0}));
}

// ---------------------------------------------------------------------------
// LinearOrdering::Build -- contrasted against TopologicalOrdering
// ---------------------------------------------------------------------------

// Same scrambled chain as LinearizesAScrambledChain, but under
// LinearOrdering: no sort at all, so ToSorted is the identity permutation
// -- joint 0 keeps sorted position 0 even though its true parent (joint 2)
// hasn't been assigned a row yet. IsForwardSafe() catches exactly this.
TEST(LinearOrderingSingleArchetype, LeavesJointOrderUntouched) {
  std::vector<std::size_t> tree_structure = {2, kNoParent, 1};
  std::vector<ArchetypeJointHandle> root_parents = {
      ArchetypeJointHandle{kNoParent, kNoParent}
  };
  std::vector<ArchetypeTreeStructure> tree = {
      MakeArchetype(true, tree_structure, root_parents)
  };

  OrderingPlan plan = LinearOrdering::Build(tree, 1);
  JointLayout layout = plan.Layout();

  EXPECT_EQ(layout.ToSorted({0, 0}), 0U);
  EXPECT_EQ(layout.ToSorted({0, 1}), 1U);
  EXPECT_EQ(layout.ToSorted({0, 2}), 2U);
  EXPECT_FALSE(plan.Topology().IsForwardSafe());
}

// Same two-archetype dependency as PlacesShallowerDepthArchetypesFirst, but
// under LinearOrdering: archetypes keep `tree`'s own order regardless of
// which one depends on which, so the dependent archetype (tree index 0)
// keeps the *smaller* row range even though it points at the archetype
// placed after it.
TEST(LinearOrderingMultipleArchetypes, KeepsTreeOrderRegardlessOfDependency) {
  std::vector<std::size_t> dependent_tree = {kNoParent};
  std::vector<ArchetypeJointHandle> dependent_roots = {
      ArchetypeJointHandle{1, 0}
  };

  std::vector<std::size_t> root_tree = {kNoParent};
  std::vector<ArchetypeJointHandle> root_roots = {
      ArchetypeJointHandle{kNoParent, kNoParent}
  };

  std::vector<ArchetypeTreeStructure> tree = {
      MakeArchetype(false, dependent_tree, dependent_roots),
      MakeArchetype(true, root_tree, root_roots)
  };

  OrderingPlan plan = LinearOrdering::Build(tree, 1);
  JointLayout layout = plan.Layout();

  EXPECT_LT(layout.ToSorted({0, 0}), layout.ToSorted({1, 0}));
  EXPECT_FALSE(plan.Topology().IsForwardSafe());
}

// Padding still applies under LinearOrdering exactly as it does under
// TopologicalOrdering -- Consolidate's lane-rounding is policy-agnostic.
TEST(LinearOrderingSingleArchetype, StillPadsToLaneMultiple) {
  std::vector<std::size_t> tree_structure = {kNoParent};
  std::vector<ArchetypeJointHandle> root_parents(
      3, ArchetypeJointHandle{kNoParent, kNoParent}
  );
  std::vector<ArchetypeTreeStructure> tree = {
      MakeArchetype(true, tree_structure, root_parents)
  };

  OrderingPlan plan = LinearOrdering::Build(tree, 4);
  JointLayout layout = plan.Layout();

  EXPECT_EQ(layout.PaddedSize(), 4U);
  EXPECT_TRUE(layout.IsPadding(3));
}

// Same as StillPadsToLaneMultiple above, at lane_size 32 -- see
// TopologicalOrderingSingleArchetype.PadsToLaneMultipleOfThirtyTwo for why
// (TESTING.md 9.4: lane sizes 4, 8, and 32 must all be exercised).
TEST(LinearOrderingSingleArchetype, PadsToLaneMultipleOfThirtyTwo) {
  std::vector<std::size_t> tree_structure = {kNoParent};
  std::vector<ArchetypeJointHandle> root_parents(
      5, ArchetypeJointHandle{kNoParent, kNoParent}
  );
  std::vector<ArchetypeTreeStructure> tree = {
      MakeArchetype(true, tree_structure, root_parents)
  };

  OrderingPlan plan = LinearOrdering::Build(tree, 32);
  JointLayout layout = plan.Layout();

  ASSERT_EQ(layout.PaddedSize(), 32U);
  for (std::size_t i = 0; i < 5; ++i) {
    EXPECT_FALSE(layout.IsPadding(i));
  }
  for (std::size_t i = 5; i < 32; ++i) {
    EXPECT_TRUE(layout.IsPadding(i));
  }
}

// ---------------------------------------------------------------------------
// OrderingPlan -- sizing and allocation
// ---------------------------------------------------------------------------

namespace {

// One root archetype, three instances of a 3-joint tree (joints 1 and 2
// both children of joint 0), lane_size 2. Consolidate lays it out one
// joint position at a time, each RoundUpToLane(3, 2) == 4 rows wide (3
// real + 1 padding): PaddedSize() == 12.
OrderingPlan BuildThreeInstancePlan() {
  static const std::vector<std::size_t> kTreeStructure = {kNoParent, 0, 0};
  static const std::vector<ArchetypeJointHandle> kRootParents(
      3, ArchetypeJointHandle{kNoParent, kNoParent}
  );
  std::vector<ArchetypeTreeStructure> tree = {
      MakeArchetype(true, kTreeStructure, kRootParents)
  };
  return TopologicalOrdering::Build(tree, 2);
}

}  // namespace

TEST(OrderingPlan, ViewInstanceCountRoundsBaseRowUpToLaneMultiple) {
  OrderingPlan plan = BuildThreeInstancePlan();

  EXPECT_EQ(plan.Layout().PaddedSize(), 12U);
  EXPECT_EQ(plan.Layout().BaseRowIndex(), 12U);
  // BaseRowIndex() + 1 == 13, rounded up to lane_size 2 == 14.
  EXPECT_EQ(plan.ViewInstanceCount(), 14U);
}

// The topology covers only real + padding rows -- never the base row or
// the lane-rounding tail -- so a traversal never visits the base row as an
// ordinary target.
TEST(OrderingPlan, TopologySizeExcludesBaseRow) {
  OrderingPlan plan = BuildThreeInstancePlan();

  EXPECT_EQ(plan.Topology().Size(), plan.Layout().PaddedSize());
}

// Allocate copies the plan into the Arena: the persisted views answer
// exactly what the plan's own views do, and fit the budget AllocatedBytes
// promised (Arena asserts on overflow).
TEST(OrderingPlan, AllocateMatchesThePlansOwnViews) {
  OrderingPlan plan = BuildThreeInstancePlan();
  Arena arena(plan.AllocatedBytes(), alignof(std::size_t));

  OrderingPlan::Allocated allocated = plan.Allocate(arena);
  JointLayout expected = plan.Layout();

  ASSERT_EQ(allocated.topology.Size(), plan.Topology().Size());
  for (std::size_t row = 0; row < allocated.topology.Size(); ++row) {
    EXPECT_EQ(allocated.topology[row], plan.Topology()[row]) << "row " << row;
    EXPECT_EQ(allocated.layout.IsPadding(row), expected.IsPadding(row));
  }
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      EXPECT_EQ(allocated.layout.ToSorted({i, j}), expected.ToSorted({i, j}));
    }
  }
}

// Every real row round-trips through ToPhysical/ToSorted.
TEST(OrderingPlan, ToSortedInvertsToPhysical) {
  OrderingPlan plan = BuildThreeInstancePlan();
  JointLayout layout = plan.Layout();

  for (std::size_t row = 0; row < layout.PaddedSize(); ++row) {
    if (!layout.IsPadding(row)) {
      EXPECT_EQ(layout.ToSorted(layout.ToPhysical(row)), row) << "row " << row;
    }
  }
}

// ---------------------------------------------------------------------------
// Precondition death tests -- one per reachable assert (TESTING.md 9.1).
// Both Build entry points forward to the same shared Consolidate, so
// lane_size gets one death test per entry point (9.2: distinct call
// shapes); SortTree/ArchetypeDepths/the self-dependency check are only
// reachable through TopologicalOrdering::Build, since LinearOrdering
// never calls them.
// ---------------------------------------------------------------------------

TEST(TopologicalOrderingBuildPrecondition, DiesOnZeroLaneSize) {
  std::vector<std::size_t> tree_structure = {kNoParent};
  std::vector<ArchetypeJointHandle> root_parents = {
      ArchetypeJointHandle{kNoParent, kNoParent}
  };
  std::vector<ArchetypeTreeStructure> tree = {
      MakeArchetype(true, tree_structure, root_parents)
  };

  EXPECT_DEATH(
      TopologicalOrdering::Build(tree, 0), "lane_size must be positive"
  );
}

TEST(LinearOrderingBuildPrecondition, DiesOnZeroLaneSize) {
  std::vector<std::size_t> tree_structure = {kNoParent};
  std::vector<ArchetypeJointHandle> root_parents = {
      ArchetypeJointHandle{kNoParent, kNoParent}
  };
  std::vector<ArchetypeTreeStructure> tree = {
      MakeArchetype(true, tree_structure, root_parents)
  };

  EXPECT_DEATH(LinearOrdering::Build(tree, 0), "lane_size must be positive");
}

// tree_structure[0] names parent index 5, but n == 1 -- out of range.
TEST(TopologicalOrderingBuildPrecondition, DiesOnOutOfRangeParentIndex) {
  std::vector<std::size_t> tree_structure = {5};
  std::vector<ArchetypeJointHandle> root_parents = {
      ArchetypeJointHandle{kNoParent, kNoParent}
  };
  std::vector<ArchetypeTreeStructure> tree = {
      MakeArchetype(true, tree_structure, root_parents)
  };

  EXPECT_DEATH(
      TopologicalOrdering::Build(tree, 1), "names an out-of-range parent"
  );
}

// Two joints, both flagged as roots (kNoParent) -- every parent index is
// still in range, isolating the root-count check from the out-of-range one.
TEST(TopologicalOrderingBuildPrecondition, DiesOnMoreThanOneLocalRoot) {
  std::vector<std::size_t> tree_structure = {kNoParent, kNoParent};
  std::vector<ArchetypeJointHandle> root_parents = {
      ArchetypeJointHandle{kNoParent, kNoParent}
  };
  std::vector<ArchetypeTreeStructure> tree = {
      MakeArchetype(true, tree_structure, root_parents)
  };

  EXPECT_DEATH(
      TopologicalOrdering::Build(tree, 1),
      "own root\\) may have no local parent"
  );
}

// joint0 is the one reachable root; joints 2 and 3 form a 2-cycle that
// joint0's own reachable chain never touches -- every parent index is in
// range and there's exactly one root, isolating the reachability check.
TEST(TopologicalOrderingBuildPrecondition, DiesOnUnreachableCycle) {
  std::vector<std::size_t> tree_structure = {kNoParent, 0, 3, 2};
  std::vector<ArchetypeJointHandle> root_parents = {
      ArchetypeJointHandle{kNoParent, kNoParent}
  };
  std::vector<ArchetypeTreeStructure> tree = {
      MakeArchetype(true, tree_structure, root_parents)
  };

  EXPECT_DEATH(
      TopologicalOrdering::Build(tree, 1), "cycle or an unreachable joint"
  );
}

// Two archetypes, both flagged is_root_archetype -- ArchetypeDepths runs
// before any per-archetype tree_structure is even looked at, so this is
// reachable with otherwise-trivial archetypes.
TEST(TopologicalOrderingBuildPrecondition, DiesOnMoreThanOneRootArchetype) {
  std::vector<std::size_t> tree_structure = {kNoParent};
  std::vector<ArchetypeJointHandle> root_parents = {
      ArchetypeJointHandle{kNoParent, kNoParent}
  };
  std::vector<ArchetypeTreeStructure> tree = {
      MakeArchetype(true, tree_structure, root_parents),
      MakeArchetype(true, tree_structure, root_parents)
  };

  EXPECT_DEATH(TopologicalOrdering::Build(tree, 1), "must be flagged");
}

// Archetype 1 (not the root) has an instance whose root_parents attaches
// to another instance of that same archetype -- exactly the "two instances
// of one archetype in a row" case ArchetypeDepths can't assign a single
// depth to.
TEST(TopologicalOrderingBuildPrecondition, DiesOnArchetypeSelfDependency) {
  std::vector<std::size_t> root_tree = {kNoParent};
  std::vector<ArchetypeJointHandle> root_roots = {
      ArchetypeJointHandle{kNoParent, kNoParent}
  };

  std::vector<std::size_t> self_dep_tree = {kNoParent};
  // Global instance numbering: archetype 0 (root) is instance 0; this
  // archetype's own two instances are global 1 and 2.
  std::vector<ArchetypeJointHandle> self_dep_roots = {
      ArchetypeJointHandle{
          0, 0
      },  // instance 0: valid, attaches to the real root
      ArchetypeJointHandle{
          1, 0
      }  // instance 1: attaches to this archetype's own instance 0
  };

  std::vector<ArchetypeTreeStructure> tree = {
      MakeArchetype(true, root_tree, root_roots),
      MakeArchetype(false, self_dep_tree, self_dep_roots)
  };

  EXPECT_DEATH(TopologicalOrdering::Build(tree, 1), "depends on");
}

// ---------------------------------------------------------------------------
// PairOrdering
// ---------------------------------------------------------------------------

namespace {

using achilles::domain::PairTopology;
using achilles::engine::topology::PairFilterLike;
using achilles::engine::topology::PairOrdering;

// Admits every pair: only PairOrdering's own structural rules apply.
struct AdmitAll {
  static std::size_t Key(
      const ArchetypeTreeStructure& /*archetype*/, std::size_t joint
  ) {
    return joint;
  }
  static bool Admits(std::size_t /*a*/, std::size_t /*b*/) { return true; }
};
static_assert(PairFilterLike<AdmitAll>);

// Admits no pair that has joint 1 in it.
struct ExcludeJointOne {
  static std::size_t Key(
      const ArchetypeTreeStructure& /*archetype*/, std::size_t joint
  ) {
    return joint;
  }
  static bool Admits(std::size_t a, std::size_t b) { return a != 1 && b != 1; }
};

// One instance of a root joint with four children: every child's row
// against every later child's, never against the root (their parent).
struct Star {
  std::vector<std::size_t> tree_structure = {kNoParent, 0, 0, 0, 0};
  std::vector<ArchetypeJointHandle> root_parents = {
      ArchetypeJointHandle{kNoParent, kNoParent}
  };
  std::vector<ArchetypeTreeStructure> tree = {
      MakeArchetype(true, tree_structure, root_parents)
  };
};

// Every real pair of `plan`, as (source row, partner row) -- the rows of
// the row ordering the pairs were planned over.
std::vector<std::pair<std::size_t, std::size_t>> PairsOf(
    const OrderingPlan& plan, std::span<const std::size_t> partners
) {
  std::vector<std::pair<std::size_t, std::size_t>> pairs;
  const JointTopology sources = plan.Topology();
  for (std::size_t row = 0; row < sources.Size(); ++row) {
    if (partners[row] != PairTopology::kNoPartner) {
      pairs.emplace_back(sources[row], partners[row]);
    }
  }
  return pairs;
}

}  // namespace

static_assert(OrderingPolicyLike<PairOrdering<TopologicalOrdering, AdmitAll>>);

TEST(PairOrderingBuild, PairsEveryNonParentRowAboveTheSourceOnce) {
  Star star;
  constexpr std::size_t kLanes = 4;
  const OrderingPlan rows = TopologicalOrdering::Build(star.tree, kLanes);
  const OrderingPlan plan =
      PairOrdering<TopologicalOrdering, AdmitAll>::Build(star.tree, kLanes);
  Arena arena(4096, alignof(std::max_align_t));
  const auto allocated = plan.Allocate(arena);

  const auto pairs = PairsOf(plan, allocated.partners);
  // C(4, 2) child pairs; the root pairs with none of its children.
  ASSERT_EQ(pairs.size(), 6U);
  const JointLayout layout = rows.Layout();
  const std::size_t root = layout.ToSorted({0, 0});
  for (const auto& [source, partner] : pairs) {
    EXPECT_LT(source, partner);
    EXPECT_NE(source, root);
    EXPECT_NE(partner, root);
    EXPECT_FALSE(layout.IsPadding(partner));
  }
  // One real pair per row: one row per pair, in the plan's one instance.
  EXPECT_EQ(plan.Layout().InstanceCount(), 1U);
  EXPECT_EQ(plan.Layout().InstanceOffsets().back(), 6U);
}

TEST(
    PairOrderingBuild, EveryLaneOfAGroupSharesItsSourceAndLinesUpWithItsPartner
) {
  Star star;
  constexpr std::size_t kLanes = 4;
  const OrderingPlan plan =
      PairOrdering<TopologicalOrdering, AdmitAll>::Build(star.tree, kLanes);
  Arena arena(4096, alignof(std::max_align_t));
  const auto allocated = plan.Allocate(arena);
  const JointTopology sources = plan.Topology();

  ASSERT_EQ(sources.Size() % kLanes, 0U);
  for (std::size_t group = 0; group < sources.Size() / kLanes; ++group) {
    bool any = false;
    for (std::size_t lane = 0; lane < kLanes; ++lane) {
      const std::size_t row = (group * kLanes) + lane;
      EXPECT_EQ(sources[row], sources[group * kLanes]);
      const std::size_t partner = allocated.partners[row];
      if (partner != PairTopology::kNoPartner) {
        any = true;
        EXPECT_EQ(partner % kLanes, lane);
        EXPECT_TRUE(plan.Layout().ToPhysical(row).instance_index == 0);
      } else {
        EXPECT_TRUE(plan.Layout().IsPadding(row));
      }
    }
    EXPECT_TRUE(any) << "a kept group has at least one real pair";
  }
}

TEST(PairOrderingBuild, DropsEveryPairTheFilterRulesOut) {
  Star star;
  const OrderingPlan rows = TopologicalOrdering::Build(star.tree, 4);
  const OrderingPlan plan =
      PairOrdering<TopologicalOrdering, ExcludeJointOne>::Build(star.tree, 4);
  Arena arena(4096, alignof(std::max_align_t));
  const auto allocated = plan.Allocate(arena);

  const auto pairs = PairsOf(plan, allocated.partners);
  const std::size_t one = rows.Layout().ToSorted({0, 1});
  // C(3, 2): the children other than joint 1.
  ASSERT_EQ(pairs.size(), 3U);
  for (const auto& [source, partner] : pairs) {
    EXPECT_NE(source, one);
    EXPECT_NE(partner, one);
  }
}

TEST(PairOrderingBuild, NoPairsGivesAnEmptyPlan) {
  std::vector<std::size_t> tree_structure = {kNoParent, 0};
  std::vector<ArchetypeJointHandle> root_parents = {
      ArchetypeJointHandle{kNoParent, kNoParent}
  };
  std::vector<ArchetypeTreeStructure> tree = {
      MakeArchetype(true, tree_structure, root_parents)
  };
  // A parent and its child: never a pair.
  const OrderingPlan plan =
      PairOrdering<TopologicalOrdering, AdmitAll>::Build(tree, 4);
  EXPECT_EQ(plan.Topology().Size(), 0U);
  EXPECT_EQ(plan.Layout().InstanceOffsets().back(), 0U);
}

// ---------------------------------------------------------------------------
// ListOrdering
// ---------------------------------------------------------------------------

namespace {

using achilles::domain::ArchetypeAttachment;
using achilles::domain::SegmentedTopology;
using achilles::engine::topology::ListOrdering;
using achilles::engine::topology::TopologyOfT;

struct Cameras {
  static constexpr std::string_view kListName = "camera";
};
struct Lights {
  static constexpr std::string_view kListName = "light";
};
using CamerasAndLights = ListOrdering<
    TopologicalOrdering,
    achilles::util::TypeList<Cameras, Lights>>;

// Two instances of a two-joint arm, each carrying a camera on its tip, a
// light on its base and a second camera on its base.
struct ArmsWithSensors {
  std::vector<std::size_t> tree_structure = {kNoParent, 0};
  std::vector<ArchetypeJointHandle> root_parents = {
      ArchetypeJointHandle{kNoParent, kNoParent},
      ArchetypeJointHandle{kNoParent, kNoParent}
  };
  std::vector<ArchetypeAttachment> attachments = {
      ArchetypeAttachment{"camera", 1, "tip_camera", {}},
      ArchetypeAttachment{"light", 0, "lamp", {}},
      ArchetypeAttachment{"camera", 0, "base_camera", {}},
  };
  std::vector<ArchetypeTreeStructure> tree = {ArchetypeTreeStructure{
      true, tree_structure, root_parents, attachments, {}
  }};
};

}  // namespace

static_assert(std::is_same_v<TopologyOfT<CamerasAndLights>, SegmentedTopology>);
static_assert(std::is_same_v<TopologyOfT<TopologicalOrdering>, JointTopology>);

TEST(
    ListOrderingBuild, NumbersEachListsEntriesAsOneSegmentLinkedToTheirJoints
) {
  ArmsWithSensors arms;
  constexpr std::size_t kLanes = 4;
  const OrderingPlan joints = TopologicalOrdering::Build(arms.tree, kLanes);
  const OrderingPlan plan = CamerasAndLights::Build(arms.tree, kLanes);
  const JointLayout layout = plan.Layout();
  const JointTopology links = plan.Topology();

  // Four cameras, then two lights: two instances times each list's own.
  ASSERT_EQ(layout.InstanceCount(), 2U);
  EXPECT_EQ(layout.InstanceOffsets()[1], 4U);
  EXPECT_EQ(layout.InstanceOffsets()[2], 6U);
  // Instance by instance, in declaration order within each list.
  const JointLayout joint_layout = joints.Layout();
  EXPECT_EQ(links[0], joint_layout.ToSorted({0, 1}));  // tip_camera, arm 0
  EXPECT_EQ(links[1], joint_layout.ToSorted({0, 0}));  // base_camera, arm 0
  EXPECT_EQ(links[2], joint_layout.ToSorted({1, 1}));
  EXPECT_EQ(links[3], joint_layout.ToSorted({1, 0}));
  EXPECT_EQ(links[4], joint_layout.ToSorted({0, 0}));  // lamp, arm 0
  EXPECT_EQ(links[5], joint_layout.ToSorted({1, 0}));
  // Padded to the lane width, the padding linked to the base row.
  ASSERT_EQ(links.Size() % kLanes, 0U);
  for (std::size_t row = 6; row < links.Size(); ++row) {
    EXPECT_TRUE(layout.IsPadding(row));
    EXPECT_EQ(links[row], joint_layout.BaseRowIndex());
  }
}

TEST(ListOrderingBuild, NoEntriesGivesNoRows) {
  std::vector<std::size_t> tree_structure = {kNoParent};
  std::vector<ArchetypeJointHandle> root_parents = {
      ArchetypeJointHandle{kNoParent, kNoParent}
  };
  std::vector<ArchetypeTreeStructure> tree = {
      MakeArchetype(true, tree_structure, root_parents)
  };
  const OrderingPlan plan = CamerasAndLights::Build(tree, 4);
  EXPECT_EQ(plan.Topology().Size(), 0U);
}
