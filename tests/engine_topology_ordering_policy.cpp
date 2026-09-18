#include <gtest/gtest.h>

#include <cstddef>
#include <span>
#include <vector>
#include <xsimd/xsimd.hpp>

#include "domain/archetype.hpp"
#include "engine/memory/arena.hpp"
#include "engine/topology/layout.hpp"
#include "engine/topology/ordering_policy.hpp"

using achilles::domain::ArchetypeJointHandle;
using achilles::domain::ArchetypeTreeStructure;
using achilles::engine::memory::Arena;
using achilles::engine::topology::Layout;
using achilles::engine::topology::LinearOrdering;
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
  static Layout Build(
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

  Layout layout = TopologicalOrdering::Build(tree, 4);
  Arena arena(layout.TopologyBytes(), alignof(std::size_t));
  auto topology = layout.AllocateTopology(arena);

  for (std::size_t i = 0; i < 3; ++i) {
    std::size_t row = layout.ToSorted(0, i);
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

  Layout layout = TopologicalOrdering::Build(tree, 1);

  EXPECT_EQ(layout.ToSorted(0, 1), 0U);  // joint1: the root, sorted first
  EXPECT_EQ(layout.ToSorted(0, 2), 1U);  // joint2: joint1's child
  EXPECT_EQ(layout.ToSorted(0, 0), 2U);  // joint0: joint2's child
  EXPECT_TRUE(layout.IsForwardSafe());
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

  Layout layout = TopologicalOrdering::Build(tree, 1);

  EXPECT_EQ(layout.ToSorted(0, 0), 0U);
  EXPECT_EQ(layout.ToSorted(0, 1), 1U);
  EXPECT_EQ(layout.ToSorted(0, 2), 2U);
  EXPECT_TRUE(layout.IsForwardSafe());
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

  Layout layout = TopologicalOrdering::Build(tree, 2);

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

  Layout layout = TopologicalOrdering::Build(tree, 32);

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
  // number: AllocateTopology below builds a real JointTopology, whose own
  // constructor checks batch safety against the *actual*
  // xsimd::batch<float>::size, not whatever lane_size this Build call was
  // given. Each archetype's block is only lane_size rows wide (2 instances,
  // 1 joint), so a lane_size smaller than the real batch width would let
  // the root's block and the dependent's block share one real batch --
  // exactly the cross-archetype case being tested here.
  const std::size_t lane_size = xsimd::batch<float>::size;
  Layout layout = TopologicalOrdering::Build(tree, lane_size);
  Arena arena(layout.TopologyBytes(), alignof(std::size_t));
  auto topology = layout.AllocateTopology(arena);

  EXPECT_EQ(topology[layout.ToSorted(1, 0)], layout.ToSorted(0, 0));
  EXPECT_EQ(topology[layout.ToSorted(1, 1)], layout.ToSorted(0, 1));
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

  Layout layout = TopologicalOrdering::Build(tree, 1);

  EXPECT_LT(layout.ToSorted(1, 0), layout.ToSorted(0, 0));
  EXPECT_TRUE(layout.IsForwardSafe());
}

// A 3-archetype chain (root -> mid -> leaf) with non-lane-aligned instance
// counts (5, 6, 10 against lane_size 8) and scattered, non-parallel
// cross-archetype attachments -- close to a real hosted-archetype shape,
// and specifically the case batch-safety (see JointTopology::
// CheckBatchSafety, asserted inside AllocateTopology below) has to hold
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

  Layout layout = TopologicalOrdering::Build(tree, 8);
  Arena arena(layout.TopologyBytes(), alignof(std::size_t));
  // Doesn't die: JointTopology's own constructor asserts CheckBatchSafety
  // over exactly the rows this hands it, so reaching this line at all is
  // the proof that no parent/child pair landed in the same SIMD batch.
  auto topology = layout.AllocateTopology(arena);

  EXPECT_TRUE(layout.IsForwardSafe());

  // Spot-check a couple of resolved cross-archetype rows against the hand-
  // computed instance mapping above.
  EXPECT_EQ(
      topology[layout.ToSorted(1, std::size_t{0} * 3)], layout.ToSorted(0, 0)
  );  // mid instance0 -> root instance0
  EXPECT_EQ(
      topology[layout.ToSorted(1, std::size_t{1} * 3)], layout.ToSorted(0, 1)
  );  // mid instance1 -> root instance1
  // leaf instance0's target global instance is 5 + (0*3)%6 == 5, which is
  // mid's own *local* instance 0 (mid's instances start at global index 5)
  // -- not mid's local instance 5.
  EXPECT_EQ(
      topology[layout.ToSorted(2, std::size_t{0} * 2)],
      layout.ToSorted(1, std::size_t{0} * 3 + 0)
  );
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

  Layout layout = LinearOrdering::Build(tree, 1);

  EXPECT_EQ(layout.ToSorted(0, 0), 0U);
  EXPECT_EQ(layout.ToSorted(0, 1), 1U);
  EXPECT_EQ(layout.ToSorted(0, 2), 2U);
  EXPECT_FALSE(layout.IsForwardSafe());
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

  Layout layout = LinearOrdering::Build(tree, 1);

  EXPECT_LT(layout.ToSorted(0, 0), layout.ToSorted(1, 0));
  EXPECT_FALSE(layout.IsForwardSafe());
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

  Layout layout = LinearOrdering::Build(tree, 4);

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

  Layout layout = LinearOrdering::Build(tree, 32);

  ASSERT_EQ(layout.PaddedSize(), 32U);
  for (std::size_t i = 0; i < 5; ++i) {
    EXPECT_FALSE(layout.IsPadding(i));
  }
  for (std::size_t i = 5; i < 32; ++i) {
    EXPECT_TRUE(layout.IsPadding(i));
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

  EXPECT_DEATH(TopologicalOrdering::Build(tree, 0), "lane_size must be positive");
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
