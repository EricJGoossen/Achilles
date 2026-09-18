#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <utility>
#include <vector>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/sim_config.hpp"
#include "domain/archetype.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/topology/layout.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "support/aba_reference.hpp"
#include "util/simd_ops.hpp"

// aba::Step wires the three Propagate*Op passes (each seeding its own
// base-row state) into a real ABAView, ABATopology and (now that
// engine::pass::TreeTraversal takes a Stride -- see aba_step.hpp) a real,
// multi-joint-capable engine::memory::SimAllocator. Every ABA Op is always
// called with TargetScalar = MathematicalT = xsimd::batch<float>, so a
// View index here is always a batch-GROUP index (Stride raw rows per
// group), never a raw row -- every helper below that turns a real
// Layout::ToSorted() row into a View index divides by `lane` for exactly
// this reason.
//
// TopologicalOrdering (ABAFieldTraits' own choice for every ABA field, see
// aba_data.hpp) guarantees no parent/child pair ever shares a lane group
// (Layout::IsForwardSafe), which is what makes a real multi-joint tree
// safe to batch at all -- that guarantee, plus Stride, is what a hand-fed
// fixture had no way to provide, and why the multi-joint tests below were
// blocked until both existed.

using namespace achilles::algorithms;
using namespace achilles::algorithms::aba;
using achilles::domain::Archetype;
using achilles::domain::ArchetypeJointHandle;
using achilles::domain::ArchetypeTreeStructure;
using achilles::engine::memory::SimAllocator;
using achilles::engine::topology::TopologicalOrdering;
using achilles::test_support::AbaInertiaOutputs;
using achilles::test_support::AbaVelocityOutputs;
using achilles::test_support::AccumulateAbaInertia;
using achilles::test_support::ComputeAbaAcceleration;
using achilles::test_support::ComputeAbaVelocity;
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

// A one-archetype, one-instance-per-joint sim: `tree_structure` describes
// that single instance's real joints (parent-per-joint,
// ArchetypeTreeStructure::kNoParent for the instance's own root). Every ABA
// field's own TopologicalOrdering pads a single instance up to a full lane,
// so Layout::ToSorted(0, j) always comes out as an exact multiple of
// Lane() here -- BatchGroup() below turns that back into the View index
// ABA's batched Store/Load actually need (see this file's header comment).
SimAllocator<ABAAlgorithm> MakeSim(std::vector<std::size_t> tree_structure) {
  std::vector<ArchetypeJointHandle> root_parents = {ArchetypeJointHandle{0, 0}};
  std::array<Archetype, 1> archetypes = {Archetype(
      "chain", std::move(tree_structure), std::move(root_parents), true, {}
  )};
  return SimAllocator<ABAAlgorithm>(archetypes);
}

std::size_t BatchGroup(
    const achilles::engine::topology::Layout& layout, std::size_t joint
) {
  return layout.ToSorted(0, joint) / Lane();
}

void PopulateRevoluteZJoint(
    ABAView& view,
    std::size_t group,
    B q0,
    Velocity qd = Velocity::Zero(),
    Force tau = Force(Vector3::Zero(), Vector3::Zero())
) {
  using MaskB = achilles::util::MaskStorageFor<MathematicalT>;

  view.Store<ABAField::kJointSubspace, B>(group, RevoluteZSubspace());
  view.Store<ABAField::kJointActivationMask, MaskB>(group, DOF0ActiveMask());
  view.Store<ABAField::kFixedJointTransform, B>(group, Transform::Identity());
  view.Store<ABAField::kRigidBodyInertia, B>(group, SimpleInertia());
  view.Store<ABAField::kJointPosition, B>(
      group, Vector6(q0, B(0.0F), B(0.0F), B(0.0F), B(0.0F), B(0.0F))
  );
  view.Store<ABAField::kJointVelocity, B>(group, qd);
  view.Store<ABAField::kJointTorque, B>(group, tau);
}

// A single real joint -- exercised by AtRestStaysAtRest/
// NonzeroJointAngleRotatesWorldTransform/RepeatedStepsProduceConsistentResults
// below.
SimAllocator<ABAAlgorithm> MakeSingleJointSim() {
  return MakeSim({ArchetypeTreeStructure::kNoParent});
}

}  // namespace

// At rest (q=0, qd=0), no gravity (a_base=0) and no applied torque: a
// system already in equilibrium must stay in equilibrium. Every velocity/
// acceleration-family output is exactly zero (see the per-Op tests in
// algorithms_aba_aba_ops.cpp for why: Rotate/Apply/CrossForce are all
// linear in their spatial-vector argument, so a zero input propagates to
// a zero output through every stage), and pose passes through unchanged.
TEST(AbaStep, AtRestStaysAtRest) {
  SimAllocator<ABAAlgorithm> sim = MakeSingleJointSim();
  ABAView view = sim.ViewFor<ABAAlgorithm>();
  std::size_t group = BatchGroup(sim.LayoutFor<TopologicalOrdering>(), 0);
  PopulateRevoluteZJoint(view, group, B(0.0F));

  SimConfig sim_config;
  ABAStep::Step(view, sim.SimContext(), sim_config, 0.0F);

  EXPECT_TRUE(BatchApprox(
      view.Load<ABAField::kWorldTransform, B>(group).Translation(),
      Transform::Identity().Translation()
  ));
  EXPECT_TRUE(BatchTrue(view.Load<ABAField::kSpatialVelocity, B>(group).IsZero()
  ));
  EXPECT_TRUE(
      BatchTrue(view.Load<ABAField::kJointAcceleration, B>(group).IsZero())
  );
  EXPECT_TRUE(
      BatchTrue(view.Load<ABAField::kSpatialAcceleration, B>(group).IsZero())
  );
}

// A nonzero joint angle must show up as a nonzero rotation in the
// propagated world transform -- confirms the full Step (seed -> velocity
// -> inertia -> acceleration passes, wired through a real ABAView and
// ABATopology) actually carries kinematic state through, not just that
// it runs without crashing.
TEST(AbaStep, NonzeroJointAngleRotatesWorldTransform) {
  SimAllocator<ABAAlgorithm> sim = MakeSingleJointSim();
  ABAView view = sim.ViewFor<ABAAlgorithm>();
  std::size_t group = BatchGroup(sim.LayoutFor<TopologicalOrdering>(), 0);
  PopulateRevoluteZJoint(view, group, B(0.3F));

  SimConfig sim_config;
  ABAStep::Step(view, sim.SimContext(), sim_config, 0.0F);

  Transform world = view.Load<ABAField::kWorldTransform, B>(group);
  EXPECT_FALSE(achilles::util::AllTrue(world.Rotation().IsIdentity()));
}

// Two real joints: joint 1 is the child of joint 0, joint 0's parent is the
// reserved base row. Joint 0 gets a nonzero own joint velocity; joint 1's
// own contribution is zero, so -- since its fixed_joint_transform/joint
// angle are both identity -- its resulting spatial velocity must come out
// exactly equal to joint 0's (ComputeAbaVelocity, called directly here with
// joint 0's own real v as v_parent, is the reference: with an identity
// x_up, x_up.Inverse().Apply(v_parent) == v_parent exactly). If cross-joint
// propagation were broken (e.g. joint 1 read the reserved base row's zero-
// seeded velocity instead of joint 0's real output, or the traversal's own
// Stride/group math were off), joint 1's velocity would come out wrong or
// zero instead. Also proves PropagateInertiaOp's backward pass folds joint
// 1's contribution into joint 0's own articulated inertia/bias force (which
// already holds joint 0's own leaf value from the forward pass, not zero)
// the same way
// PropagateInertiaOpTest.AccumulatesIntoParentOutputsRatherThanOverwriting
// proves the += contract directly -- this is that same contract exercised
// through a real tree traversal instead of a direct call.
TEST(AbaStep, PropagatesThroughTwoJointChain) {
  SimAllocator<ABAAlgorithm> sim =
      MakeSim({ArchetypeTreeStructure::kNoParent, 0});
  ABAView view = sim.ViewFor<ABAAlgorithm>();
  const auto& layout = sim.LayoutFor<TopologicalOrdering>();
  std::size_t group0 = BatchGroup(layout, 0);
  std::size_t group1 = BatchGroup(layout, 1);

  Velocity qd0(Vector3(B(0.5F), B(0.0F), B(0.0F)), Vector3::Zero());
  PopulateRevoluteZJoint(view, group0, B(0.0F), qd0);
  PopulateRevoluteZJoint(view, group1, B(0.0F));

  SimConfig sim_config;
  ABAStep::Step(view, sim.SimContext(), sim_config, 0.0F);

  // Reference: joint 0 reads the base row (identity transform, zero
  // velocity, per the default SimConfig); joint 1 reads joint 0's own real
  // output as its v_parent.
  AbaVelocityOutputs joint0 =
      ComputeAbaVelocity(Transform::Identity(), Velocity::Zero(), B(0.0F), qd0);
  AbaVelocityOutputs joint1 =
      ComputeAbaVelocity(joint0.x_world, joint0.v, B(0.0F));
  ASSERT_FALSE(achilles::util::AllTrue(joint0.v.IsZero()))
      << "Test premise violated: joint 0's own qd produced zero velocity, "
         "so this test can't distinguish real propagation from a broken "
         "(always-zero) parent read.";

  EXPECT_TRUE(
      BatchApprox(view.Load<ABAField::kSpatialVelocity, B>(group1), joint0.v)
  );
  EXPECT_TRUE(
      BatchApprox(view.Load<ABAField::kSpatialVelocity, B>(group1), joint1.v)
  );

  InertiaOperator<false> i_a_parent = joint0.i_a;
  Force p_parent = joint0.p;
  AccumulateAbaInertia(
      joint1, Force(Vector3::Zero(), Vector3::Zero()), i_a_parent, p_parent
  );

  EXPECT_TRUE(BatchApprox(
      view.Load<ABAField::kArticulatedInertia, B>(group0).AsMatrix(),
      i_a_parent.AsMatrix()
  ));
  EXPECT_TRUE(BatchApprox(
      view.Load<ABAField::kArticulatedBiasForce, B>(group0), p_parent
  ));
}

// Two real joints, both parented to joint 0, the sole real root. After the
// backward (inertia) pass, joint 0's articulated inertia and bias force
// must equal its own leaf value (from the forward pass) plus both
// children's individually-transformed contributions, folded in via two
// separate += calls -- extends
// PropagateInertiaOpTest.AccumulatesIntoParentOutputsRatherThanOverwriting
// (which proves the += contract with two *direct* calls onto a synthetic
// accumulator) to prove the same thing actually happens when the traversal
// itself drives two real sibling joints into the same real parent row.
TEST(AbaStep, AccumulatesMultipleChildrenIntoSharedParent) {
  SimAllocator<ABAAlgorithm> sim =
      MakeSim({ArchetypeTreeStructure::kNoParent, 0, 0});
  ABAView view = sim.ViewFor<ABAAlgorithm>();
  const auto& layout = sim.LayoutFor<TopologicalOrdering>();
  std::size_t group0 = BatchGroup(layout, 0);
  std::size_t group1 = BatchGroup(layout, 1);
  std::size_t group2 = BatchGroup(layout, 2);

  PopulateRevoluteZJoint(view, group0, B(0.0F));
  PopulateRevoluteZJoint(view, group1, B(0.0F));
  PopulateRevoluteZJoint(view, group2, B(0.0F));

  SimConfig sim_config;
  ABAStep::Step(view, sim.SimContext(), sim_config, 0.0F);

  AbaVelocityOutputs joint0 =
      ComputeAbaVelocity(Transform::Identity(), Velocity::Zero(), B(0.0F));
  AbaVelocityOutputs joint1 =
      ComputeAbaVelocity(joint0.x_world, joint0.v, B(0.0F));
  AbaVelocityOutputs joint2 =
      ComputeAbaVelocity(joint0.x_world, joint0.v, B(0.0F));

  InertiaOperator<false> i_a_parent = joint0.i_a;
  Force p_parent = joint0.p;
  Force zero_tau(Vector3::Zero(), Vector3::Zero());
  AccumulateAbaInertia(joint1, zero_tau, i_a_parent, p_parent);
  AccumulateAbaInertia(joint2, zero_tau, i_a_parent, p_parent);

  EXPECT_TRUE(BatchApprox(
      view.Load<ABAField::kArticulatedInertia, B>(group0).AsMatrix(),
      i_a_parent.AsMatrix()
  ));
  EXPECT_TRUE(BatchApprox(
      view.Load<ABAField::kArticulatedBiasForce, B>(group0), p_parent
  ));
}

// root -> child -> grandchild, at least three real levels deep. With
// nonzero gravity (a_base, seeded via PropagateAccelerationOp::Initialize)
// and every joint at rest (q=0, qd=0), the deepest joint's acceleration
// must reflect gravity having propagated through every intervening level
// -- proves the full backward-then-forward recursion (not just a single
// hop to/from the reserved base row, which is all the single-joint tests
// above exercise). The reference chain below calls the same three real Ops
// directly, in the same order ABAStep::Step's own passes run them, so this
// is an exact comparison, not just a "did anything change at all" check.
TEST(AbaStep, PropagatesThroughMultiLevelTree) {
  SimAllocator<ABAAlgorithm> sim =
      MakeSim({ArchetypeTreeStructure::kNoParent, 0, 1});
  ABAView view = sim.ViewFor<ABAAlgorithm>();
  const auto& layout = sim.LayoutFor<TopologicalOrdering>();
  std::size_t group0 = BatchGroup(layout, 0);
  std::size_t group1 = BatchGroup(layout, 1);
  std::size_t group2 = BatchGroup(layout, 2);

  PopulateRevoluteZJoint(view, group0, B(0.0F));
  PopulateRevoluteZJoint(view, group1, B(0.0F));
  PopulateRevoluteZJoint(view, group2, B(0.0F));

  Acceleration gravity(Vector3::Zero(), Vector3(B(0.0F), B(0.0F), B(-9.8F)));
  SimConfig sim_config;
  sim_config.base_acceleration = gravity;
  ABAStep::Step(view, sim.SimContext(), sim_config, 0.0F);

  // Forward (velocity) pass: at rest, every level's own v/c/p is zero, but
  // I_A/x_up/x_world are still real per-joint values the backward/forward
  // passes below need.
  AbaVelocityOutputs joint0 =
      ComputeAbaVelocity(Transform::Identity(), Velocity::Zero(), B(0.0F));
  AbaVelocityOutputs joint1 =
      ComputeAbaVelocity(joint0.x_world, joint0.v, B(0.0F));
  AbaVelocityOutputs joint2 =
      ComputeAbaVelocity(joint1.x_world, joint1.v, B(0.0F));

  // Backward (inertia) pass, deepest first -- joint2 folds into joint1,
  // then joint1 (already carrying joint2's contribution) folds into joint0.
  Force zero_tau(Vector3::Zero(), Vector3::Zero());
  InertiaOperator<false> i_a1 = joint1.i_a;
  Force p1 = joint1.p;
  AbaInertiaOutputs inertia2 = AccumulateAbaInertia(joint2, zero_tau, i_a1, p1);

  InertiaOperator<false> i_a0 = joint0.i_a;
  Force p0 = joint0.p;
  AbaVelocityOutputs joint1_with_child = joint1;
  joint1_with_child.i_a = i_a1;
  joint1_with_child.p = p1;
  AbaInertiaOutputs inertia1 =
      AccumulateAbaInertia(joint1_with_child, zero_tau, i_a0, p0);

  InertiaOperator<false> i_a_base = InertiaOperator<false>::Zero();
  Force p_base = Force::Zero();
  AbaVelocityOutputs joint0_with_children = joint0;
  joint0_with_children.i_a = i_a0;
  joint0_with_children.p = p0;
  AbaInertiaOutputs inertia0 =
      AccumulateAbaInertia(joint0_with_children, zero_tau, i_a_base, p_base);

  // Forward (acceleration) pass, root first.
  Acceleration a0 =
      ComputeAbaAcceleration(inertia0, joint0.x_up, joint0.c, gravity);
  Acceleration a1 = ComputeAbaAcceleration(inertia1, joint1.x_up, joint1.c, a0);
  Acceleration a2 = ComputeAbaAcceleration(inertia2, joint2.x_up, joint2.c, a1);

  ASSERT_FALSE(achilles::util::AllTrue(a2.IsZero()))
      << "Test premise violated: gravity produced zero acceleration at the "
         "deepest joint, so this test can't distinguish real propagation "
         "from a broken (gravity-never-arrives) traversal.";
  EXPECT_TRUE(
      BatchApprox(view.Load<ABAField::kSpatialAcceleration, B>(group2), a2)
  );
}

// PropagateInertiaOp accumulates (+=) into I_A_parent_out/p_parent_out
// (see aba_ops.cpp) rather than overwriting them, which is correct for
// folding multiple children's contributions into one parent *within* a
// single Step() call -- but that only works long-term if the reserved
// base row's kArticulatedInertia/kArticulatedBiasForce fields are reset
// before each pass starts, or successive Step() calls would silently
// accumulate onto whatever the previous call left behind. That reset is
// PropagateInertiaOp's own Initialize (aba_ops.cpp: `*I_A_base_out =
// kIABase; *p_base_out = kPBase;`, both Zero()) -- the same per-Op
// Initialize mechanism PropagateVelocityOp/PropagateAccelerationOp use to
// seed their own base-row reads (see the header comment on aba_ops.hpp).
// RunPass calls it once, unconditionally, at the base row before every
// single Apply pass (engine/pass/algorithm_step.hpp), so it runs on every
// Step() call, not just the first -- resolving what used to be an open
// question without needing aba::Step, the allocator, or the simulation
// loop to take on that responsibility themselves.
TEST(AbaStep, RepeatedStepsProduceConsistentResults) {
  SimAllocator<ABAAlgorithm> sim = MakeSingleJointSim();
  ABAView view = sim.ViewFor<ABAAlgorithm>();
  std::size_t group = BatchGroup(sim.LayoutFor<TopologicalOrdering>(), 0);
  PopulateRevoluteZJoint(view, group, B(0.3F));

  SimConfig sim_config;
  ABAStep::Step(view, sim.SimContext(), sim_config, 0.0F);
  Transform world_after_first = view.Load<ABAField::kWorldTransform, B>(group);
  Acceleration accel_after_first =
      view.Load<ABAField::kSpatialAcceleration, B>(group);

  // Re-populate the same joint state (Step() itself mutates kJointPosition
  // nowhere, but this mirrors a real simulation loop re-driving the same
  // input each tick) and step again -- if the base row's I_A/p carried
  // over from the first call instead of being zeroed, this second call's
  // articulated-inertia accumulation would start from nonzero leftovers
  // and diverge from the first call's result.
  PopulateRevoluteZJoint(view, group, B(0.3F));
  ABAStep::Step(view, sim.SimContext(), sim_config, 0.0F);

  EXPECT_TRUE(BatchApprox(
      view.Load<ABAField::kWorldTransform, B>(group).Translation(),
      world_after_first.Translation()
  ));
  EXPECT_TRUE(BatchApprox(
      view.Load<ABAField::kSpatialAcceleration, B>(group), accel_after_first
  ));
}
