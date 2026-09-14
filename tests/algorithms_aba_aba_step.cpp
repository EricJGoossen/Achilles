#include <gtest/gtest.h>
#include <xsimd/xsimd.hpp>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_ops.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/conventions.hpp"
#include "support/joint_topology_fixture.hpp"
#include "support/planar_view_fixture.hpp"
#include "util/simd_ops.hpp"

// aba::Step wires the three Propagate*Op passes (each seeding its own base-
// a real ABAView/ABATopology -- these tests are deliberately small and use
// exactly one real joint (Size() == 1, parent = the reserved base row),
// sidestepping a design question this file's tests can't answer on their
// own: JointTopology's CheckBatchSafety reasons about per-JOINT indices
// (data_'s entries, one per real joint), but OpInvoker passes
// TreeTraversal's raw, unscaled traversal step directly as the "index"
// argument to PlanarView::Load<xsimd::batch<float>>, which for a batched
// TargetScalar addresses storage in units of
// xsimd::batch<float>::size-sized GROUPS, not one raw row at a time (see
// PlanarView's own comment on Value/Load: "instance index directly when T
// is scalar, batch index when T is batched"). Since every ABA Op is
// always called with TargetScalar = MathematicalT = xsimd::batch<float>,
// what a single TreeTraversal step actually addresses in a real,
// more-than-one-joint ABAView -- one joint's data replicated/batched some
// other way, or xsimd::batch<float>::size consecutive *different* joints
// -- isn't something this file resolves. That's exactly the kind of
// question the top-to-bottom correctness file needs to settle; flagging
// it here rather than guessing at a multi-joint test that might exercise
// the wrong granularity.

using namespace achilles::algorithms;
using namespace achilles::algorithms::aba;
using achilles::test_support::FixtureFor;
using achilles::test_support::TopologyFixture;

namespace {

using B = MathematicalT;
using Fixture = FixtureFor<ABAView>::Type;

// PlanarView::Load/Store<xsimd::batch<float>>(index) address storage in
// units of xsimd::batch<float>::size *raw* rows per index (see this
// file's header comment) -- so a view addressed at batch-group indices 0
// and 1 (this file's "joint" and "base" rows) needs at least
// 2 * batch-width raw instances allocated, not 2. Every value stored
// below is a uniform broadcast across lanes (B(x) broadcasts x to every
// lane), so which "extra" raw rows within a group end up under which
// lane is irrelevant here -- they all hold the identical scenario.
constexpr std::size_t kNumBatchGroups = 2;
std::size_t RequiredInstances() {
  return kNumBatchGroups * xsimd::batch<float>::size;
}

::testing::AssertionResult BatchTrue(const auto& mask) {
  if (achilles::util::AllTrue(mask)) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << "not all lanes true";
}

// One real joint (row 0), one reserved base row (row 1) -- topology[0] =
// 1 is out of Size() (1)'s range, the documented "no parent" convention,
// so CheckBatchSafety's parent-lane check skips it regardless of
// xsimd::batch<float>::size. See domain_topology_joint_topology.cpp for
// the general form this is a special case of.
TopologyFixture SingleJointTopology() { return {{{1}}, {{0, 0}}}; }

void PopulateRevoluteZJoint(ABAView& view, MathematicalT q0) {
  using MaskB = achilles::util::MaskStorageFor<MathematicalT>;

  Matrix6x6 s = Matrix6x6::Zero();
  s(2, 0) = B(1.0F);
  view.Store<ABAField::kJointSubspace, B>(0, s);
  view.Store<ABAField::kJointActivationMask, MaskB>(
      0, Mat6Mask(true, false, false, false, false, false)
  );
  view.Store<ABAField::kFixedJointTransform, B>(0, Transform::Identity());
  view.Store<ABAField::kRigidBodyInertia, B>(
      0,
      Inertia(B(2.0F), Vector3::Zero(), B(2.0F), B(3.0F), B(4.0F), B(0.0F), B(0.0F), B(0.0F))
  );
  view.Store<ABAField::kJointPosition, B>(
      0, Vector6(q0, B(0.0F), B(0.0F), B(0.0F), B(0.0F), B(0.0F))
  );
  view.Store<ABAField::kJointVelocity, B>(0, Velocity::Zero());
  view.Store<ABAField::kJointTorque, B>(0, Force(Vector3::Zero(), Vector3::Zero()));
}

}  // namespace

// At rest (q=0, qd=0), no gravity (a_base=0) and no applied torque: a
// system already in equilibrium must stay in equilibrium. Every velocity/
// acceleration-family output is exactly zero (see the per-Op tests in
// algorithms_aba_aba_ops.cpp for why: Rotate/Apply/CrossForce are all
// linear in their spatial-vector argument, so a zero input propagates to
// a zero output through every stage), and pose passes through unchanged.
TEST(AbaStep, AtRestStaysAtRest) {
  Fixture fixture(RequiredInstances());
  ABAView view = fixture.MakeView();
  TopologyFixture topo = SingleJointTopology();
  PopulateRevoluteZJoint(view, B(0.0F));

  Step(
      view, topo.topology,
      Transform::Identity(), Velocity::Zero(), Acceleration::Zero()
  );

  EXPECT_TRUE(BatchTrue(
      view.Load<ABAField::kWorldTransform, B>(0).Translation().IsApprox(
          Transform::Identity().Translation()
      )
  ));
  EXPECT_TRUE(BatchTrue(view.Load<ABAField::kSpatialVelocity, B>(0).IsZero()));
  EXPECT_TRUE(BatchTrue(view.Load<ABAField::kJointAcceleration, B>(0).IsZero()));
  EXPECT_TRUE(BatchTrue(view.Load<ABAField::kSpatialAcceleration, B>(0).IsZero()));
}

// A nonzero joint angle must show up as a nonzero rotation in the
// propagated world transform -- confirms the full Step (seed -> velocity
// -> inertia -> acceleration passes, wired through a real ABAView and
// ABATopology) actually carries kinematic state through, not just that
// it runs without crashing.
TEST(AbaStep, NonzeroJointAngleRotatesWorldTransform) {
  Fixture fixture(RequiredInstances());
  ABAView view = fixture.MakeView();
  TopologyFixture topo = SingleJointTopology();
  PopulateRevoluteZJoint(view, B(0.3F));

  Step(
      view, topo.topology,
      Transform::Identity(), Velocity::Zero(), Acceleration::Zero()
  );

  Transform world = view.Load<ABAField::kWorldTransform, B>(0);
  EXPECT_FALSE(achilles::util::AllTrue(world.Rotation().IsIdentity()));
}

// -- Coverage blocked on the real allocator --
//
// Building a topology with more than one *real* joint related to each
// other (a genuine parent/child pair, not "one joint plus the reserved
// base row") means laying out their PlanarView rows so no parent/child
// pair ever shares a SIMD lane group -- that's JointTopology's own
// CheckBatchSafety contract (domain/topology/joint_topology.hpp), and
// building memory that satisfies it is exactly the allocator's job, not
// a test fixture's (see the porting-plan comment on PlanarViewFixture in
// tests/support/planar_view_fixture.hpp -- this file must not grow its
// own topological-sort/padding logic to fake that). These are left
// skipped, not written against a guessed-at layout, until the allocator
// exists to build one for real.

TEST(AbaStep, PropagatesThroughTwoJointChain) {
  GTEST_SKIP() << "Needs the real allocator to lay out two related real "
                  "joints (a genuine parent/child pair) batch-safely.";
  // Two real joints: joint 1 is the child of joint 0, joint 0's parent is
  // the reserved base row. Verify PropagateVelocityOp's v_out for joint 1
  // correctly incorporates joint 0's own v_out as its parent velocity
  // (cross-joint propagation, not just within one joint's own inputs --
  // algorithms_aba_aba_ops.cpp's PropagateVelocityOpTest only exercises
  // one joint's formula in isolation), and that PropagateInertiaOp's
  // backward pass folds joint 1's contribution into joint 0's articulated
  // inertia/bias force (I_A_parent_out/p_parent_out) the same way
  // PropagateInertiaOpTest.AccumulatesIntoParentOutputsRatherThanOverwriting
  // proves the +=  contract in isolation -- this is that same contract
  // exercised through a real tree traversal instead of a direct call.
}

TEST(AbaStep, AccumulatesMultipleChildrenIntoSharedParent) {
  GTEST_SKIP() << "Needs the real allocator to lay out two sibling real "
                  "joints (both children of one parent) batch-safely.";
  // Two real joints, both parented to the same third joint. After the
  // backward (inertia) pass, the parent's articulated inertia and bias
  // force must equal the sum of both children's individually-transformed
  // contributions -- extends
  // PropagateInertiaOpTest.AccumulatesIntoParentOutputsRatherThanOverwriting
  // (which proves the += contract with two *direct* calls) to prove the
  // same thing actually happens when the traversal itself drives two
  // real sibling joints into the same parent row.
}

TEST(AbaStep, PropagatesThroughMultiLevelTree) {
  GTEST_SKIP() << "Needs the real allocator to lay out a 3+ level real "
                  "joint chain batch-safely.";
  // root -> child -> grandchild, at least three real levels deep. With
  // nonzero gravity (a_base, seeded via PropagateAccelerationOp::Initialize)
// and every joint at rest (q=0,
  // now safe since Quaternion::Exp's zero-rotation fix), the deepest
  // joint's acceleration must be nonzero and reflect gravity having
  // propagated through every intervening level -- proves the full
  // backward-then-forward recursion (not just a single hop to/from the
  // reserved base row, which is all the current passing tests exercise).
}

// -- Coverage blocked on an unresolved design decision, not the allocator
// --
//
// PropagateInertiaOp accumulates (+=) into I_A_parent_out/p_parent_out
// (see aba_ops.cpp) rather than overwriting them, which is correct for
// folding multiple children's contributions into one parent *within* a
// single Step() call -- but nothing in aba::Step, or anywhere else,
// resets the reserved base row's kArticulatedInertia/
// kArticulatedBiasForce fields before a pass starts. Every real joint's
// own I_A is reset unconditionally each forward (velocity) pass
// (`*I_A_out = I.AsArticulated();`, an assignment not an accumulation),
// so real joints self-correct every Step() call regardless of prior
// state -- but the base row is never a *target* of that write, only ever
// a += destination for whichever joints are rooted there. The tests
// above (and everywhere else in this session) only pass because
// PlanarViewFixture hands out freshly zeroed memory every time. Whether
// the base row's accumulator fields should be zeroed by aba::Step itself
// at the start of every call, by the allocator once at setup, or by the
// simulation loop between steps is a real design decision -- not
// something to default silently.
TEST(AbaStep, RepeatedStepsProduceConsistentResults) {
  GTEST_SKIP() << "Needs a decision on who is responsible for zeroing the "
                  "reserved base row's accumulator fields between Step() "
                  "calls -- see the comment above this test.";
  // Call Step() twice in a row over the same view/topology (simulating
  // two consecutive timesteps) with identical joint state both times.
  // The second call's results must equal the first's -- if the base
  // row's I_A/p fields aren't reset between calls, the second call's
  // articulated-inertia accumulation silently starts from the first
  // call's leftover values instead of from zero, and this would fail.
}
