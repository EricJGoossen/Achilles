#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <utility>
#include <vector>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/sim_config.hpp"
#include "algorithms/viz/viz_data.hpp"
#include "algorithms/viz/viz_step.hpp"
#include "domain/archetype.hpp"
#include "domain/math/vector3.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/topology/layout.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "support/aba_reference.hpp"
#include "util/simd_ops.hpp"

// VizAlgorithm has no Step of its own (viz_step.hpp names engine::NoStep),
// so there is no traversal/Op to exercise the way algorithms_aba_aba_step.
// cpp or algorithms_vi_vi_step.cpp do -- what actually needs proving here
// is the memory-sharing contract itself:
//   1. kWorldTransform really is the same block ABA's own kWorldTransform
//      is (WorldTransformSlot, shared_slots.hpp) -- a value ABAStep
//      writes must be visible through a VizView without VizAlgorithm ever
//      writing it itself.
//   2. kVisualExtents/kVisualColor are populated from archetype data by
//      name ("visual_extents"/"visual_color"), the same SimAllocator::
//      Populate pass every other named field goes through.
//   3. A joint whose archetype gives no visual_extents reads back exactly
//      Vector3::Zero() -- the "draw nothing" default the viz_data.hpp
//      header comment promises.

using namespace achilles::algorithms;
using namespace achilles::algorithms::viz;
using achilles::domain::Archetype;
using achilles::domain::ArchetypeField;
using achilles::domain::ArchetypeJointHandle;
using achilles::domain::ArchetypeTreeStructure;
using achilles::engine::memory::SimAllocator;
using achilles::engine::topology::TopologicalOrdering;
using achilles::test_support::DOF0ActiveMask;
using achilles::test_support::RevoluteZSubspace;
using achilles::test_support::SimpleInertia;

namespace {

using B = MathematicalT;

std::size_t Lane() { return xsimd::batch<float>::size; }

::testing::AssertionResult BatchTrue(const auto& mask) {
  if (achilles::util::AllTrue(mask)) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << "not all lanes true";
}

::testing::AssertionResult BatchApprox(const auto& lhs, const auto& rhs) {
  return BatchTrue(lhs.IsApprox(rhs));
}

std::size_t BatchGroup(
    const achilles::engine::topology::Layout& layout, std::size_t joint
) {
  return layout.ToSorted(0, joint) / Lane();
}

// One archetype, one real joint, hosting both ABAAlgorithm and
// VizAlgorithm -- the shape a real sim (algorithms::RegisteredAlgorithms)
// actually uses, needed here so kWorldTransform's shared slot has a real
// writer (ABAStep) to prove VizView reads from.
SimAllocator<aba::ABAAlgorithm, VizAlgorithm> MakeSharedSim(
    std::vector<ArchetypeField> fields = {}
) {
  std::vector<ArchetypeJointHandle> root_parents = {ArchetypeJointHandle{0, 0}};
  std::array<Archetype, 1> archetypes = {Archetype(
      "joint",
      {ArchetypeTreeStructure::kNoParent},
      std::move(root_parents),
      true,
      std::move(fields)
  )};
  return SimAllocator<aba::ABAAlgorithm, VizAlgorithm>(archetypes);
}

void PopulateRevoluteZJoint(aba::ABAView& view, std::size_t group) {
  using MaskB = achilles::util::MaskStorageFor<MathematicalT>;

  view.Store<aba::ABAField::kJointSubspace, B>(group, RevoluteZSubspace());
  view.Store<aba::ABAField::kJointActivationMask, MaskB>(
      group, DOF0ActiveMask()
  );
  view.Store<aba::ABAField::kFixedJointTransform, B>(
      group, Transform::Identity()
  );
  view.Store<aba::ABAField::kRigidBodyInertia, B>(group, SimpleInertia());
  Vector6 q_coords(B(0.3F), B(0.0F), B(0.0F), B(0.0F), B(0.0F), B(0.0F));
  Transform x_joint = Transform::Exp(Velocity(RevoluteZSubspace() * q_coords));
  view.Store<aba::ABAField::kJointPosition, B>(group, x_joint);
  view.Store<aba::ABAField::kJointVelocity, B>(group, Velocity::Zero());
  view.Store<aba::ABAField::kJointTorque, B>(
      group, Force(Vector3::Zero(), Vector3::Zero())
  );
}

}  // namespace

// Proves the SharedAs contract end to end: after a real ABAStep::Step call
// writes kWorldTransform through ABAView, the exact same value must be
// readable through a completely independent VizView -- if VizField::
// kWorldTransform carved its own private block instead of aliasing
// WorldTransformSlot, this would read back the zero-seeded default
// (Transform::Identity()'s translation, but a degenerate/never-updated
// rotation) instead.
TEST(VizData, WorldTransformAliasesAbaOwnWorldTransform) {
  SimAllocator<aba::ABAAlgorithm, VizAlgorithm> sim = MakeSharedSim();
  aba::ABAView aba_view = sim.ViewFor<aba::ABAAlgorithm>();
  VizView viz_view = sim.ViewFor<VizAlgorithm>();
  std::size_t group = BatchGroup(sim.LayoutFor<TopologicalOrdering>(), 0);

  PopulateRevoluteZJoint(aba_view, group);

  SimConfig sim_config;
  aba::ABAStep::Step(sim.SimContext(), sim_config, 0.0F);

  Transform expected = aba_view.Load<aba::ABAField::kWorldTransform, B>(group);
  ASSERT_FALSE(achilles::util::AllTrue(expected.Rotation().IsIdentity()))
      << "Test premise violated: a nonzero joint angle produced an "
         "identity world rotation, so this test can't distinguish a real "
         "shared read from a stale default.";

  Transform actual = viz_view.Load<VizField::kWorldTransform, B>(group);
  EXPECT_TRUE(BatchApprox(actual.Translation(), expected.Translation()));
  EXPECT_TRUE(BatchTrue(actual.Rotation() == expected.Rotation()));
}

// kVisualExtents/kVisualColor are populated from archetype data by name,
// the same as any other named field (e.g. ABA's own kJointSubspace) --
// this is SimAllocator::Populate exercised through VizField instead.
//
// Loaded at scalar (not batched) shape, by the field's own raw sorted row
// (not a batch group): the archetype supplies only one real joint, so
// every other lane in that joint's lane group is an untouched, zero-seeded
// padding row -- a batched Load here would compare a whole lane's worth of
// (mostly padding) values against one uniform broadcast expectation and
// fail for the wrong reason. View's own scalar Load/Store addresses by raw
// row directly (see view.hpp's comment on Value/Load), which is exactly
// the one real row this test cares about.
TEST(VizData, VisualExtentsAndColorPopulateFromArchetypeByName) {
  std::vector<ArchetypeField> fields = {
      ArchetypeField{"visual_extents", 3, {0.5, 0.25, 0.1}},
      ArchetypeField{"visual_color", 3, {1.0, 0.0, 0.0}},
  };
  SimAllocator<aba::ABAAlgorithm, VizAlgorithm> sim =
      MakeSharedSim(std::move(fields));
  VizView view = sim.ViewFor<VizAlgorithm>();
  std::size_t row = sim.LayoutFor<TopologicalOrdering>().ToSorted(0, 0);

  using ScalarVector3 = achilles::domain::math::Vector3<float>;
  ScalarVector3 extents = view.Load<VizField::kVisualExtents, float>(row);
  ScalarVector3 color = view.Load<VizField::kVisualColor, float>(row);

  EXPECT_TRUE(extents.IsApprox(ScalarVector3(0.5F, 0.25F, 0.1F)));
  EXPECT_TRUE(color.IsApprox(ScalarVector3(1.0F, 0.0F, 0.0F)));
}

// A joint whose archetype gives no visual_extents at all must read back
// exactly Vector3::Zero() -- SimAllocator::Carve's own ZeroFill, left
// untouched since Populate only ever overwrites a row for a field an
// archetype actually names (see FieldVisitor::Populate's own comment,
// sim_allocator.hpp). This is the "draw nothing" default a renderer relies
// on (see viz_data.hpp's own header comment).
TEST(VizData, MissingVisualExtentsDefaultsToZero) {
  SimAllocator<aba::ABAAlgorithm, VizAlgorithm> sim = MakeSharedSim();
  VizView view = sim.ViewFor<VizAlgorithm>();
  std::size_t group = BatchGroup(sim.LayoutFor<TopologicalOrdering>(), 0);

  Vector3 extents = view.Load<VizField::kVisualExtents, B>(group);
  EXPECT_TRUE(BatchTrue(extents.IsZero()));
}
