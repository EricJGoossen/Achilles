#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include "domain/archetype.hpp"
#include "domain/math/vector3.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/field_contract.hpp"
#include "engine/memory/binding.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/topology/layout.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "util/tmp.hpp"

using achilles::domain::Archetype;
using achilles::domain::ArchetypeField;
using achilles::domain::ArchetypeJointHandle;
using achilles::domain::ArchetypeTreeStructure;
using achilles::domain::math::Vector3;
using achilles::domain::math::Vector3Assembler;
using achilles::engine::Algorithm;
using achilles::engine::memory::SimAllocator;
using achilles::engine::memory::SimAllocatorForT;
using achilles::engine::topology::LinearOrdering;
using achilles::engine::topology::PlanarLayout;
using achilles::engine::topology::TopologicalOrdering;
using achilles::util::TypeList;

namespace {

// Cross-algorithm shared-slot tag -- both SharedField's and SecondField's
// own field below name this, so SimAllocator must carve it exactly once
// and hand both Algorithms the same underlying block.
struct SharedInputSlot;

enum class SharedField : std::uint8_t {
  kSharedInput,   // aliased with SecondField::kSharedAgain via SharedAs
  kPrivateInput,  // populated from the archetype, not shared
  kOutput,        // never named by any archetype -- algorithm-computed
  kCount,
};

template <SharedField F>
struct SharedFieldTraits;

template <>
struct SharedFieldTraits<SharedField::kSharedInput> {
  using Assembler = Vector3Assembler<float>;
  using SharedAs = SharedInputSlot;
  using Ordering = TopologicalOrdering;
  using Layout = PlanarLayout;
  static constexpr std::string_view kName = "shared_input";
};
template <>
struct SharedFieldTraits<SharedField::kPrivateInput> {
  using Assembler = Vector3Assembler<float>;
  using Ordering = TopologicalOrdering;
  using Layout = PlanarLayout;
  static constexpr std::string_view kName = "private_input";
};
template <>
struct SharedFieldTraits<SharedField::kOutput> {
  using Assembler = Vector3Assembler<float>;
  using Ordering = TopologicalOrdering;
  using Layout = PlanarLayout;
};
static_assert(achilles::engine::
                  FieldTraitsLike<SharedField, SharedFieldTraits>);

enum class SecondField : std::uint8_t { kSharedAgain, kCount };

template <SecondField F>
struct SecondFieldTraits;

template <>
struct SecondFieldTraits<SecondField::kSharedAgain> {
  using Assembler = Vector3Assembler<float>;
  using SharedAs = SharedInputSlot;
  using Ordering = TopologicalOrdering;
  using Layout = PlanarLayout;
};
static_assert(achilles::engine::
                  FieldTraitsLike<SecondField, SecondFieldTraits>);

// One root archetype, 2 instances, a 2-joint tree (joint 1 is the child of
// joint 0) -- enough to exercise real placement, padding, and the shared
// base row without hardcoding a SIMD width.
Archetype MakeArmArchetype() {
  std::vector<std::size_t> tree_structure = {
      ArchetypeTreeStructure::kNoParent, 0
  };
  std::vector<ArchetypeJointHandle> root_parents = {
      ArchetypeJointHandle{0, 0}, ArchetypeJointHandle{1, 0}
  };
  std::vector<ArchetypeField> fields;
  // instance i, joint j, leaf l -> 100*i + 10*j + l (every value distinct).
  fields.push_back(ArchetypeField{
      "shared_input", 3, {0, 1, 2, 10, 11, 12, 100, 101, 102, 110, 111, 112}
  });
  fields.push_back(ArchetypeField{
      "private_input",
      3,
      {1000, 1001, 1002, 1010, 1011, 1012, 1100, 1101, 1102, 1110, 1111, 1112}
  });
  return {
      "arm",
      std::move(tree_structure),
      std::move(root_parents),
      true,
      std::move(fields)
  };
}

}  // namespace

TEST(SimAllocatorPopulate, CopiesArchetypeDataIntoTheRightSortedRow) {
  Archetype arm = MakeArmArchetype();
  std::array<Archetype, 1> archetypes = {std::move(arm)};

  using Sim = SimAllocator<Algorithm<SharedField, SharedFieldTraits>>;
  Sim sim(archetypes);
  auto view = sim.ViewFor<Algorithm<SharedField, SharedFieldTraits>>();

  for (std::size_t i = 0; i < 2; ++i) {
    for (std::size_t j = 0; j < 2; ++j) {
      std::size_t row =
          sim.LayoutFor<TopologicalOrdering>().ToSorted(0, (i * 2) + j);
      auto base = static_cast<float>((100 * i) + (10 * j));
      EXPECT_TRUE((view.Load<SharedField::kSharedInput, float>(row).IsApprox(
          Vector3<float>(base, base + 1.0F, base + 2.0F)
      )));
      auto private_base = static_cast<float>(1000 + (100 * i) + (10 * j));
      EXPECT_TRUE((view.Load<SharedField::kPrivateInput, float>(row).IsApprox(
          Vector3<float>(private_base, private_base + 1.0F, private_base + 2.0F)
      )));
    }
  }
}

// kOutput names no ArchetypeField (no kName at all) -- Populate must leave
// it exactly as Carve's own ZeroFill left it, not touch it.
TEST(SimAllocatorPopulate, LeavesFieldsWithNoKNameZeroed) {
  Archetype arm = MakeArmArchetype();
  std::array<Archetype, 1> archetypes = {std::move(arm)};

  using Sim = SimAllocator<Algorithm<SharedField, SharedFieldTraits>>;
  Sim sim(archetypes);
  auto view = sim.ViewFor<Algorithm<SharedField, SharedFieldTraits>>();

  std::size_t row = sim.LayoutFor<TopologicalOrdering>().ToSorted(0, 0);
  EXPECT_TRUE((view.Load<SharedField::kOutput, float>(row).IsZero()));
}

// Every field bound to TopologicalOrdering gets its padding rows seeded
// with Vector3::PaddingSeed() (== Zero(), so this alone wouldn't
// distinguish seeded-zero from never-written-zero) -- IsPadding directly
// names which row is padding, so this confirms Populate/SeedPadding agree
// with Layout on which rows are real vs padding, not just that both
// happen to be zero.
TEST(SimAllocatorSeedPadding, PaddingRowsAreDistinctFromRealRows) {
  Archetype arm = MakeArmArchetype();
  std::array<Archetype, 1> archetypes = {std::move(arm)};

  using Sim = SimAllocator<Algorithm<SharedField, SharedFieldTraits>>;
  Sim sim(archetypes);

  const auto& layout = sim.LayoutFor<TopologicalOrdering>();
  bool found_padding_row = false;
  for (std::size_t row = 0; row < layout.PaddedSize(); ++row) {
    if (layout.IsPadding(row)) {
      found_padding_row = true;
    }
  }
  // With 2 instances and lane_size == SimAllocator's own required lane
  // size, a padding row only fails to exist if the lane size divides 2
  // evenly -- assert the premise so a future lane-size change that breaks
  // it fails loudly here instead of this test silently checking nothing.
  ASSERT_TRUE(found_padding_row)
      << "Test premise violated: no padding row exists for 2 instances at "
         "this build's lane size -- widen the archetype's instance count.";
}

TEST(SimAllocatorTopology, SizeMatchesLayoutPaddedSize) {
  Archetype arm = MakeArmArchetype();
  std::array<Archetype, 1> archetypes = {std::move(arm)};

  using Sim = SimAllocator<Algorithm<SharedField, SharedFieldTraits>>;
  Sim sim(archetypes);

  EXPECT_EQ(
      sim.TopologyFor<TopologicalOrdering>().Size(),
      sim.LayoutFor<TopologicalOrdering>().PaddedSize()
  );
}

// Joint 1's resolved parent must be joint 0's own sorted row, for the same
// instance -- proves Topology() actually reflects the archetype's real
// parent/child structure, not just that it's the right size.
TEST(SimAllocatorTopology, ChildResolvesToParentsRowWithinSameInstance) {
  Archetype arm = MakeArmArchetype();
  std::array<Archetype, 1> archetypes = {std::move(arm)};

  using Sim = SimAllocator<Algorithm<SharedField, SharedFieldTraits>>;
  Sim sim(archetypes);

  const auto& layout = sim.LayoutFor<TopologicalOrdering>();
  std::size_t instance0_joint0 = layout.ToSorted(0, 0);
  std::size_t instance0_joint1 = layout.ToSorted(0, 1);
  EXPECT_EQ(
      sim.TopologyFor<TopologicalOrdering>()[instance0_joint1],
      instance0_joint0
  );
}

// SharedField::kSharedInput and SecondField::kSharedAgain both name
// SharedInputSlot -- SimAllocator must carve that block exactly once and
// hand out the same memory to both Algorithms, so data Populate wrote
// through SharedField's own kName is visible reading through
// SecondField's aliased field too.
TEST(SimAllocatorSharedAs, CrossAlgorithmFieldsAliasTheSameBlock) {
  Archetype arm = MakeArmArchetype();
  std::array<Archetype, 1> archetypes = {std::move(arm)};

  using Sim = SimAllocator<
      Algorithm<SharedField, SharedFieldTraits>,
      Algorithm<SecondField, SecondFieldTraits>>;
  Sim sim(archetypes);

  auto shared_view =
      sim.ViewFor<Algorithm<SharedField, SharedFieldTraits>>();
  auto second_view =
      sim.ViewFor<Algorithm<SecondField, SecondFieldTraits>>();

  std::size_t row = sim.LayoutFor<TopologicalOrdering>().ToSorted(
      0, (1 * 2) + 1
  );  // instance 1, joint 1
  Vector3<float> via_shared =
      shared_view.Load<SharedField::kSharedInput, float>(row);
  Vector3<float> via_second =
      second_view.Load<SecondField::kSharedAgain, float>(row);

  EXPECT_TRUE(via_shared.IsApprox(via_second));
  EXPECT_TRUE(via_shared.IsApprox(Vector3<float>(110.0F, 111.0F, 112.0F)));
}

// SimAllocatorForT<TypeList<Algorithms...>> must name the exact same type as
// SimAllocator<Algorithms...> -- proves the registered-list path (see
// algorithms/registry.hpp) is a pure unwrap, not a different type a real
// SimAllocator<Algorithms...> caller couldn't substitute for it.
static_assert(std::is_same_v<
              SimAllocatorForT<
                  TypeList<Algorithm<SharedField, SharedFieldTraits>>>,
              SimAllocator<Algorithm<SharedField, SharedFieldTraits>>>);
static_assert(std::is_same_v<
              SimAllocatorForT<TypeList<
                  Algorithm<SharedField, SharedFieldTraits>,
                  Algorithm<SecondField, SecondFieldTraits>>>,
              SimAllocator<
                  Algorithm<SharedField, SharedFieldTraits>,
                  Algorithm<SecondField, SecondFieldTraits>>>);

// Same as CrossAlgorithmFieldsAliasTheSameBlock above, but built through
// SimAllocatorForT<TypeList<...>> instead of naming SimAllocator directly --
// confirms a Sim built the "registered list" way behaves identically at
// runtime, not just that its type matches.
TEST(SimAllocatorForRegisteredList, BehavesIdenticallyToDirectSimAllocator) {
  Archetype arm = MakeArmArchetype();
  std::array<Archetype, 1> archetypes = {std::move(arm)};

  using Registered = TypeList<
      Algorithm<SharedField, SharedFieldTraits>,
      Algorithm<SecondField, SecondFieldTraits>>;
  using Sim = SimAllocatorForT<Registered>;
  Sim sim(archetypes);

  auto shared_view =
      sim.ViewFor<Algorithm<SharedField, SharedFieldTraits>>();
  auto second_view =
      sim.ViewFor<Algorithm<SecondField, SecondFieldTraits>>();

  std::size_t row = sim.LayoutFor<TopologicalOrdering>().ToSorted(
      0, (1 * 2) + 1
  );  // instance 1, joint 1
  Vector3<float> via_shared =
      shared_view.Load<SharedField::kSharedInput, float>(row);
  Vector3<float> via_second =
      second_view.Load<SecondField::kSharedAgain, float>(row);

  EXPECT_TRUE(via_shared.IsApprox(via_second));
  EXPECT_TRUE(via_shared.IsApprox(Vector3<float>(110.0F, 111.0F, 112.0F)));
}

namespace {

// One field declares TopologicalOrdering explicitly; the other declares no
// Ordering at all, so it resolves to the default, LinearOrdering. Nothing
// negotiates a single "program-wide" ordering between them -- each gets
// laid out purely from its own declared policy.
enum class MixedField : std::uint8_t { kTopological, kLinear, kCount };
template <MixedField F>
struct MixedFieldTraits;
template <>
struct MixedFieldTraits<MixedField::kTopological> {
  using Assembler = Vector3Assembler<float>;
  using Ordering = TopologicalOrdering;
  using Layout = PlanarLayout;
};
template <>
struct MixedFieldTraits<MixedField::kLinear> {
  using Assembler = Vector3Assembler<float>;
  using Layout = PlanarLayout;
};
static_assert(achilles::engine::FieldTraitsLike<MixedField, MixedFieldTraits>);
static_assert(std::is_same_v<
              achilles::engine::memory::FieldOrderingT<
                  MixedField,
                  MixedFieldTraits,
                  MixedField::kLinear>,
              LinearOrdering>);

}  // namespace

// Joint 1 is the tree's actual root and joint 0 is its child, so physical
// order (0, 1) disagrees with dependency order (1, 0). TopologicalOrdering
// sorts joint 1 (the root) first; LinearOrdering's identity placement
// leaves joint 0 first. Two fields of the *same* algorithm, over the same
// archetype, must land those joints at different rows -- proving each
// field's own Traits<F>::Ordering, not any single sim-wide choice,
// determines its physical layout.
TEST(
    SimAllocatorPerFieldOrdering,
    FieldsWithDifferentPoliciesGetIndependentRowMappings
) {
  std::vector<std::size_t> tree_structure = {
      1, ArchetypeTreeStructure::kNoParent
  };
  std::vector<ArchetypeJointHandle> root_parents = {ArchetypeJointHandle{0, 0}};
  Archetype backwards(
      "backwards", std::move(tree_structure), std::move(root_parents), true, {}
  );
  std::array<Archetype, 1> archetypes = {std::move(backwards)};

  using Sim = SimAllocator<Algorithm<MixedField, MixedFieldTraits>>;
  Sim sim(archetypes);

  std::size_t topo_row = sim.LayoutFor<TopologicalOrdering>().ToSorted(0, 0);
  std::size_t linear_row = sim.LayoutFor<LinearOrdering>().ToSorted(0, 0);
  EXPECT_NE(topo_row, linear_row);
}

// The archetype's own "private_input" field only has 3 doubles per
// (instance, joint) worth of storage error-checked against
// SharedFieldTraits::kPrivateInput's Vector3Assembler (also 3 leaves) --
// deliberately mismatching scalars_per_leaf here must trip Populate's own
// shape assert rather than silently reading past the declared count.
TEST(SimAllocatorPopulate, DiesOnArchetypeFieldScalarCountMismatch) {
  std::vector<std::size_t> tree_structure = {ArchetypeTreeStructure::kNoParent};
  std::vector<ArchetypeJointHandle> root_parents = {ArchetypeJointHandle{0, 0}};
  std::vector<ArchetypeField> fields;
  fields.push_back(ArchetypeField{"private_input", 2, {1.0, 2.0}});
  Archetype mismatched(
      "mismatched",
      std::move(tree_structure),
      std::move(root_parents),
      true,
      std::move(fields)
  );
  std::array<Archetype, 1> archetypes = {std::move(mismatched)};

  using Sim = SimAllocator<Algorithm<SharedField, SharedFieldTraits>>;
  EXPECT_DEATH(Sim sim(archetypes), "scalar count doesn't match");
}

namespace {

// Names the same SharedInputSlot tag SharedField::kSharedInput/
// SecondField::kSharedAgain already share, but with LinearOrdering (the
// default -- no Ordering declared) instead of their TopologicalOrdering --
// same Assembler, so bytes/alignment/lane count all still match; only
// ordering_id disagrees. Enough to trip Carve's own shape-mismatch assert
// without also tripping the (unrelated) byte-size mismatch case above.
enum class MismatchedOrderingField : std::uint8_t { kSharedAgain, kCount };
template <MismatchedOrderingField F>
struct MismatchedOrderingFieldTraits;
template <>
struct MismatchedOrderingFieldTraits<MismatchedOrderingField::kSharedAgain> {
  using Assembler = Vector3Assembler<float>;
  using SharedAs = SharedInputSlot;
  using Layout = PlanarLayout;
};
static_assert(achilles::engine::FieldTraitsLike<
              MismatchedOrderingField,
              MismatchedOrderingFieldTraits>);

}  // namespace

// Two algorithms name the same SharedAs tag but disagree on the shared
// field's own Ordering (TopologicalOrdering vs the default LinearOrdering)
// -- Carve's reused-slot check (engine/memory/sim_allocator.hpp) compares
// ordering_id along with bytes/alignment/lane_count precisely so this kind
// of disagreement is caught loudly instead of two algorithms silently
// aliasing memory whose row order they disagree about.
TEST(SimAllocatorSharedAs, DiesOnIncompatibleOrderingForSameSharedTag) {
  Archetype arm = MakeArmArchetype();
  std::array<Archetype, 1> archetypes = {std::move(arm)};

  using Sim = SimAllocator<
      Algorithm<SharedField, SharedFieldTraits>,
      Algorithm<MismatchedOrderingField, MismatchedOrderingFieldTraits>>;
  EXPECT_DEATH(Sim sim(archetypes), "unsound");
}
