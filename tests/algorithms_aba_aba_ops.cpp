#include <gtest/gtest.h>

#include "algorithms/aba/aba_ops.hpp"
#include "algorithms/conventions.hpp"
#include "util/simd_ops.hpp"

// These tests confirm each Op correctly wires together primitives that
// are already independently unit-tested elsewhere (Transform::Exp,
// Transform::operator*/Inverse/Apply in domain_spatial_transform.cpp;
// Matrix multiply/transpose/MaskedInverse in domain_math_matrix.cpp;
// InertiaOperator arithmetic in domain_spatial_inertia.cpp) -- right
// operands, right order, right sign. They are NOT an independent
// derivation of the ABA recursion from physical first principles (energy
// conservation, a closed-form pendulum, ...); that's the top-to-bottom
// correctness file to design separately.
//
// Every domain type here is instantiated at achilles::algorithms'
// MathematicalT (xsimd::batch<float>), the only scalar shape these Ops
// are ever actually called with in production -- not float, so there is
// no scalar/batched split to cover the way domain/math's own tests have.
// Values are broadcast uniformly across lanes and read back via lane 0;
// per-lane independence is already covered generically for the
// underlying math types (e.g. Vector3Batched, ActivationMaskBatched).

using namespace achilles::algorithms;
using namespace achilles::algorithms::aba;

namespace {

using B = MathematicalT;

::testing::AssertionResult BatchTrue(const auto& mask) {
  if (achilles::util::AllTrue(mask)) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << "not all lanes true";
}

// Row 2 (yaw / angular-Z, per Dual's Roll/Pitch/Yaw/X/Y/Z row order) is
// driven by generalized-coordinate slot 0 -- a single-DOF revolute joint
// about Z, using DOF slot 0. Matches the convention the deleted
// test_helpers.cpp used for the same joint shape.
Matrix6x6 RevoluteZSubspace() {
  Matrix6x6 s = Matrix6x6::Zero();
  s(2, 0) = B(1.0F);
  return s;
}

Mat6Mask DOF0ActiveMask() {
  return Mat6Mask(true, false, false, false, false, false);
}

// mass 2, centered (h=0), diagonal principal moments (2,3,4) -- the same
// trivially-physically-valid fixture domain_spatial_inertia.cpp uses.
Inertia SimpleInertia() {
  return {
      B(2.0F),
      Vector3::Zero(),
      B(2.0F),
      B(3.0F),
      B(4.0F),
      B(0.0F),
      B(0.0F),
      B(0.0F)
  };
}

}  // namespace

// Initialize seeds exactly the caller-configured base state -- what a
// root joint reads back as x_world_parent/v_parent (see
// PropagateVelocityOpTest below).
TEST(PropagateVelocityOpTest, InitializeSeedsConfiguredBaseState) {
  Transform x_world_base(
      Vector3(B(1.0F), B(2.0F), B(3.0F)), Quaternion::Identity()
  );
  Velocity v_base(Vector3(B(0.1F), B(0.2F), B(0.3F)), Vector3::Zero());
  PropagateVelocityOp op(x_world_base, v_base);

  Transform x_world_out;
  Velocity v_out;
  op.Initialize(&x_world_out, &v_out);

  EXPECT_TRUE(
      BatchTrue(x_world_out.Translation().IsApprox(x_world_base.Translation()))
  );
  EXPECT_TRUE(BatchTrue(v_out.IsApprox(v_base)));
}

// At q=0 (joint at rest) and x_tree=Identity, a joint contributes no
// motion of its own: pose and velocity must pass straight through from
// parent to child unchanged, and the bias acceleration (which is
// proportional to the joint's own velocity contribution) must be zero.
TEST(PropagateVelocityOpTest, AtRestPassesParentStateThroughUnchanged) {
  Matrix6x6 s = RevoluteZSubspace();
  Inertia inertia = SimpleInertia();
  Transform x_world_parent(
      Vector3(B(1.0F), B(0.0F), B(0.0F)), Quaternion::Identity()
  );
  Transform x_tree = Transform::Identity();
  Transform x_joint = Transform::Identity();
  Velocity qd = Velocity::Zero();
  Velocity v_parent(
      Vector3(B(0.0F), B(0.0F), B(0.3F)), Vector3(B(1.0F), B(0.0F), B(0.0F))
  );

  // x_world_base/v_base are irrelevant here -- this test exercises
  // operator(), not Initialize.
  PropagateVelocityOp op(Transform::Identity(), Velocity::Zero());
  InertiaOperator<false> i_a_out;
  Transform x_up_out;
  Transform x_world_out;
  Velocity v_out;
  Acceleration c_out;
  Force p_out;
  op(s,
     inertia,
     x_world_parent,
     x_tree,
     x_joint,
     qd,
     v_parent,
     &i_a_out,
     &x_up_out,
     &x_world_out,
     &v_out,
     &c_out,
     &p_out);

  EXPECT_TRUE(BatchTrue(
      x_up_out.Translation().IsApprox(Transform::Identity().Translation())
  ));
  EXPECT_TRUE(BatchTrue(
      x_world_out.Translation().IsApprox(x_world_parent.Translation())
  ));
  EXPECT_TRUE(BatchTrue(v_out.IsApprox(v_parent)));
  EXPECT_TRUE(BatchTrue(c_out.IsZero()));
  EXPECT_TRUE(BatchTrue(i_a_out.IsApprox(inertia.AsArticulated())));

  // p_out = v.CrossForce(I.Apply(v)), recomputed independently via the
  // same already-tested SpatialVelocity::CrossForce/Inertia::Apply.
  Force expected_p = v_parent.CrossForce(inertia.Apply(v_parent));
  EXPECT_TRUE(BatchTrue(p_out.IsApprox(expected_p)));
}

// Nonzero joint motion: x_up/v_out recomputed independently via
// Transform::Exp/operator*/Inverse/Apply (each already unit-tested on
// its own) instead of re-typing PropagateVelocityOp's own expressions.
TEST(
    PropagateVelocityOpTest, NonzeroJointMotionMatchesIndependentRecomputation
) {
  Matrix6x6 s = RevoluteZSubspace();
  Inertia inertia = SimpleInertia();
  Transform x_world_parent(
      Vector3(B(1.0F), B(0.0F), B(0.0F)), Quaternion::Identity()
  );
  Transform x_tree(Vector3(B(0.5F), B(0.0F), B(0.0F)), Quaternion::Identity());
  Vector6 q(B(0.3F), B(0.0F), B(0.0F), B(0.0F), B(0.0F), B(0.0F));
  Velocity joint_twist(s * q);
  Transform x_joint = Transform::Exp(joint_twist);
  Velocity qd(Vector3(B(0.0F), B(0.0F), B(0.2F)), Vector3::Zero());
  Velocity v_parent = Velocity::Zero();

  // x_world_base/v_base are irrelevant here -- this test exercises
  // operator(), not Initialize.
  PropagateVelocityOp op(Transform::Identity(), Velocity::Zero());
  InertiaOperator<false> i_a_out;
  Transform x_up_out;
  Transform x_world_out;
  Velocity v_out;
  Acceleration c_out;
  Force p_out;
  op(s,
     inertia,
     x_world_parent,
     x_tree,
     x_joint,
     qd,
     v_parent,
     &i_a_out,
     &x_up_out,
     &x_world_out,
     &v_out,
     &c_out,
     &p_out);

  Transform expected_x_up = x_tree * Transform::Exp(joint_twist);
  Velocity qd_spatial(s * qd.AsVector6());
  Velocity expected_v = expected_x_up.Inverse().Apply(v_parent) + qd_spatial;

  EXPECT_TRUE(
      BatchTrue(x_up_out.Translation().IsApprox(expected_x_up.Translation()))
  );
  EXPECT_TRUE(BatchTrue(x_up_out.Rotation().IsApprox(expected_x_up.Rotation()))
  );
  EXPECT_TRUE(BatchTrue(v_out.IsApprox(expected_v)));
  EXPECT_TRUE(BatchTrue(c_out.IsApprox(expected_v.Cross(qd_spatial))));
}

// x_up = Identity means "this joint doesn't move its child relative to
// its parent frame" -- Transform::Apply(InertiaOperator)/Apply(Force)
// under Identity are already proven to be no-ops
// (domain_spatial_transform.cpp), so I_A_parent_out/p_parent_out must
// equal exactly the un-transformed contribution terms. Isolates the
// contribution formula (U, D, D_inv, I_A - U D^-1 U^T, ...) from the
// separate "transform into parent frame" step.
TEST(
    PropagateInertiaOpTest, IdentityTransformPassesContributionThroughUnchanged
) {
  Matrix6x6 s = RevoluteZSubspace();
  Mat6Mask mask = DOF0ActiveMask();
  Transform x_up = Transform::Identity();
  InertiaOperator<false> i_a = SimpleInertia().AsArticulated();
  Acceleration c = Acceleration::Zero();
  Force tau(Vector3::Zero(), Vector3::Zero());
  Force p = Force::Zero();

  PropagateInertiaOp op;
  InertiaOperator<false> i_a_parent_out = InertiaOperator<false>::Zero();
  InertiaOperator<false> u_out;
  InertiaOperator<true> d_inv_out;
  Force p_parent_out = Force::Zero();
  Force u_out_force;
  op(s,
     mask,
     x_up,
     i_a,
     c,
     tau,
     p,
     &i_a_parent_out,
     &u_out,
     &d_inv_out,
     &p_parent_out,
     &u_out_force);

  InertiaOperator<false> expected_u(i_a.AsMatrix() * s);
  InertiaOperator<false> expected_d(s.Transpose() * expected_u.AsMatrix());
  InertiaOperator<true> expected_d_inv = expected_d.MaskedInverse(mask);
  Matrix6x6 expected_u_d_inv =
      expected_u.AsMatrix() * expected_d_inv.AsMatrix();
  InertiaOperator<false> expected_contribution =
      i_a - InertiaOperator<false>(
                expected_u_d_inv * expected_u.Transpose().AsMatrix()
            );

  EXPECT_TRUE(BatchTrue(u_out.IsApprox(expected_u)));
  EXPECT_TRUE(BatchTrue(i_a_parent_out.IsApprox(expected_contribution, 1e-3F)));
}

// Initialize always zeroes the base-row accumulator -- there's no
// caller-configurable state to pass through (unlike PropagateVelocityOp/
// PropagateAccelerationOp's Initialize), so this just pins the contract
// down directly rather than relying on it only being implied by
// AccumulatesIntoParentOutputsRatherThanOverwriting below.
TEST(PropagateInertiaOpTest, InitializeZeroesAccumulator) {
  PropagateInertiaOp op;
  InertiaOperator<false> i_a_out;
  Force p_out;
  op.Initialize(&i_a_out, &p_out);

  EXPECT_TRUE(BatchTrue(i_a_out.IsZero()));
  EXPECT_TRUE(BatchTrue(p_out.IsZero()));
}

// I_A_parent_out/p_parent_out are accumulated (+=), not overwritten --
// the mechanism that lets more than one child add its contribution into
// a shared parent row. A pre-existing nonzero value in the accumulator
// must survive, with this call's own contribution added on top.
TEST(
    PropagateInertiaOpTest, AccumulatesIntoParentOutputsRatherThanOverwriting
) {
  Matrix6x6 s = RevoluteZSubspace();
  Mat6Mask mask = DOF0ActiveMask();
  Transform x_up = Transform::Identity();
  InertiaOperator<false> i_a = SimpleInertia().AsArticulated();
  Acceleration c = Acceleration::Zero();
  Force tau(Vector3::Zero(), Vector3::Zero());
  Force p = Force::Zero();

  PropagateInertiaOp op;

  InertiaOperator<false> i_a_parent_a = InertiaOperator<false>::Zero();
  Force p_parent_a = Force::Zero();
  InertiaOperator<false> u_out;
  InertiaOperator<true> d_inv_out;
  Force u_out_force;
  op(s,
     mask,
     x_up,
     i_a,
     c,
     tau,
     p,
     &i_a_parent_a,
     &u_out,
     &d_inv_out,
     &p_parent_a,
     &u_out_force);

  InertiaOperator<false> preexisting =
      SimpleInertia().AsArticulated() * B(0.5F);
  InertiaOperator<false> i_a_parent_b = preexisting;
  Force p_parent_b = Force::Zero();
  op(s,
     mask,
     x_up,
     i_a,
     c,
     tau,
     p,
     &i_a_parent_b,
     &u_out,
     &d_inv_out,
     &p_parent_b,
     &u_out_force);

  EXPECT_TRUE(BatchTrue(i_a_parent_b.IsApprox(preexisting + i_a_parent_a, 1e-3F)
  ));
}

// Initialize seeds exactly the caller-configured base acceleration --
// what a root joint reads back as a_parent (this is how gravity enters
// the algorithm).
TEST(PropagateAccelerationOpTest, InitializeSeedsConfiguredBaseState) {
  Acceleration a_base(Vector3::Zero(), Vector3(B(0.0F), B(0.0F), B(-9.8F)));
  PropagateAccelerationOp op(a_base);

  Acceleration a_out;
  op.Initialize(&a_out);

  EXPECT_TRUE(BatchTrue(a_out.IsApprox(a_base)));
}

// a_pre/qdd recomputed independently via Transform::Inverse/Apply and
// InertiaOperator::Apply/Transpose (each already unit-tested on its own).
TEST(PropagateAccelerationOpTest, MatchesIndependentRecomputation) {
  Matrix6x6 s = RevoluteZSubspace();
  InertiaOperator<true> d_inv =
      InertiaOperator<false>(SimpleInertia().AsMatrix()).Inverse();
  InertiaOperator<false> u(SimpleInertia().AsMatrix() * s);
  Transform x_up(Vector3(B(0.2F), B(0.0F), B(0.0F)), Quaternion::Identity());
  Acceleration c(Vector3::Zero(), Vector3(B(0.0F), B(0.1F), B(0.0F)));
  Acceleration a_parent(Vector3::Zero(), Vector3(B(0.0F), B(0.0F), B(-1.0F)));
  Force u_force(Vector3::Zero(), Vector3::Zero());

  // a_base is irrelevant here -- this test exercises operator(), not
  // Initialize.
  PropagateAccelerationOp op(Acceleration::Zero());
  Acceleration qdd_out;
  Acceleration a_out;
  op(s, d_inv, u, x_up, c, a_parent, u_force, &qdd_out, &a_out);

  Acceleration expected_a_pre = x_up.Inverse().Apply(a_parent) + c;
  Acceleration expected_qdd = d_inv.Apply(Force(
      (u_force.AsVector6() - u.Transpose().Apply(expected_a_pre).AsVector6())
  ));
  Acceleration expected_a =
      expected_a_pre + Acceleration(s * expected_qdd.AsVector6());

  EXPECT_TRUE(BatchTrue(qdd_out.IsApprox(expected_qdd, 1e-3F)));
  EXPECT_TRUE(BatchTrue(a_out.IsApprox(expected_a, 1e-3F)));
}
