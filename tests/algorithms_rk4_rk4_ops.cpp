#include <gtest/gtest.h>

#include <cmath>

#include "algorithms/conventions.hpp"
#include "algorithms/rk4/rk4_ops.hpp"
#include "util/simd_ops.hpp"

// RK4's four per-row updates, called directly:
//   BeginStepOp       (q, qd)    -> (q_n, qd_n), both sums zeroed
//   AccumulateStageOp (qd, qdd)  -> sums += weight * (qd, qdd)
//   AdvanceStageOp    (S, q_n, qd_n, qdd) + incoming qd as k_v
//                                -> next stage (q, qd)
//   CommitOp          (S, q_n, qd_n, sums) -> committed (q, qd)
// algorithms_rk4_rk4_step.cpp covers them through the real View/traversal;
// algorithms_integrators.cpp covers the whole tick, including the 1/2/2/1
// weighting's fourth-order convergence.
//
// The Ops use their outputs' incoming values differently -- accumulated
// onto (AccumulateStageOp), read as data (AdvanceStageOp's k_v), or
// overwritten (the rest) -- so each test seeds the outputs to make the
// difference observable (TESTING.md 9.2).
//
// Pose expectations use the closed form for a revolute-Z joint (rotation by
// theta about Z is (cos(theta/2), 0, 0, sin(theta/2))) rather than
// recomputing Transform::Exp.

using namespace achilles::algorithms;
using namespace achilles::algorithms::rk4;

namespace {

using B = MathematicalT;

::testing::AssertionResult BatchTrue(const auto& mask) {
  if (achilles::util::AllTrue(mask)) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << "not all lanes true";
}

Matrix6x6 RevoluteZSubspace() {
  Matrix6x6 s = Matrix6x6::Zero();
  s(2, 0) = B(1.0F);
  return s;
}

Velocity Slot0(float rate) {
  Velocity value(Vector6(B(rate), B(0.0F), B(0.0F), B(0.0F), B(0.0F), B(0.0F)));
  return value;
}

Acceleration Slot0Acceleration(float rate) {
  Acceleration value(
      Vector6(B(rate), B(0.0F), B(0.0F), B(0.0F), B(0.0F), B(0.0F))
  );
  return value;
}

Transform RotationZ(float theta) {
  return {
      Vector3::Zero(),
      Quaternion(
          B(std::cos(theta * 0.5F)), B(0.0F), B(0.0F), B(std::sin(theta * 0.5F))
      )
  };
}

Transform JunkPose() {
  return {
      Vector3(B(9.0F), B(9.0F), B(9.0F)),
      Quaternion(B(0.0F), B(1.0F), B(0.0F), B(0.0F))
  };
}

}  // namespace

// Snapshots the tick's start and zeroes both sums -- whatever the previous
// tick left in them must not leak into this one.
TEST(RK4BeginStepOpTest, SnapshotsStartStateAndZeroesBothSums) {
  Transform q_start_out = JunkPose();
  Velocity qd_start_out = Slot0(-9.0F);
  Velocity qd_sum_out = Slot0(-9.0F);
  Acceleration qdd_sum_out = Slot0Acceleration(-9.0F);

  BeginStepOp op;
  op(RotationZ(0.3F),
     Slot0(1.5F),
     &q_start_out,
     &qd_start_out,
     &qd_sum_out,
     &qdd_sum_out);

  EXPECT_TRUE(
      BatchTrue(q_start_out.Rotation().IsApprox(RotationZ(0.3F).Rotation()))
  );
  EXPECT_TRUE(BatchTrue(qd_start_out.IsApprox(Slot0(1.5F))));
  EXPECT_TRUE(BatchTrue(qd_sum_out.IsZero()));
  EXPECT_TRUE(BatchTrue(qdd_sum_out.IsZero()));
}

// Adds weight * (qd, qdd) onto the running sums, rather than overwriting
// them -- four stages fold into the same two sums.
TEST(AccumulateStageOpTest, AddsWeightedStageOntoRunningSums) {
  Velocity qd_sum_out = Slot0(1.0F);
  Acceleration qdd_sum_out = Slot0Acceleration(10.0F);

  AccumulateStageOp op(2.0F);
  op(Slot0(3.0F), Slot0Acceleration(5.0F), &qd_sum_out, &qdd_sum_out);

  EXPECT_TRUE(BatchTrue(qd_sum_out.IsApprox(Slot0(7.0F))));
  EXPECT_TRUE(BatchTrue(qdd_sum_out.IsApprox(Slot0Acceleration(20.0F))));
}

// The next stage starts from (q_n, qd_n), not from the previous stage's
// state: q = q_n * Exp(S k_v h) with k_v the INCOMING qd (the previous
// stage's velocity), and qd = qd_n + qdd h. The incoming q is junk and must
// be ignored; the incoming qd must be read before it's overwritten.
TEST(AdvanceStageOpTest, AdvancesFromTickStartUsingIncomingVelocityAsKv) {
  Transform q_out = JunkPose();
  Velocity qd_out = Slot0(2.0F);  // k_v of the previous stage.

  AdvanceStageOp op(0.5F);
  op(RevoluteZSubspace(),
     RotationZ(0.1F),
     Slot0(1.0F),
     Slot0Acceleration(4.0F),
     &q_out,
     &qd_out);

  // q = 0.1 + k_v * h = 0.1 + 2 * 0.5; qd = qd_n + qdd * h = 1 + 4 * 0.5.
  EXPECT_TRUE(BatchTrue(q_out.Rotation().IsApprox(RotationZ(1.1F).Rotation())));
  EXPECT_TRUE(BatchTrue(qd_out.IsApprox(Slot0(3.0F))));
}

// The blend divides both sums by 6 (the 1+2+2+1 weights) over the full dt:
// qd = qd_n + dt/6 * sum(qdd), q = q_n * Exp(S dt/6 * sum(qd)).
TEST(RK4CommitOpTest, BlendsSumsOverSixthsOfTheFullDt) {
  Transform q_out = JunkPose();
  Velocity qd_out = Slot0(-9.0F);

  CommitOp op(0.5F);
  op(RevoluteZSubspace(),
     RotationZ(0.1F),
     Slot0(1.0F),
     Slot0(12.0F),
     Slot0Acceleration(24.0F),
     &q_out,
     &qd_out);

  // q = 0.1 + 12 * 0.5 / 6 = 1.1; qd = 1 + 24 * 0.5 / 6 = 3.
  EXPECT_TRUE(BatchTrue(q_out.Rotation().IsApprox(RotationZ(1.1F).Rotation())));
  EXPECT_TRUE(BatchTrue(qd_out.IsApprox(Slot0(3.0F))));
}

// The committed pose accumulates across ticks, so it's renormalized.
TEST(RK4CommitOpTest, RenormalizesTheCommittedRotation) {
  Transform q_start(
      Vector3::Zero(), Quaternion(B(1.01F), B(0.0F), B(0.0F), B(0.0F))
  );
  Transform q_out;
  Velocity qd_out;

  CommitOp op(0.5F);
  op(RevoluteZSubspace(),
     q_start,
     Slot0(0.0F),
     Slot0(6.0F),
     Acceleration::Zero(),
     &q_out,
     &qd_out);

  EXPECT_TRUE(
      BatchTrue(xsimd::abs(q_out.Rotation().Norm() - B(1.0F)) < B(1e-6F))
  );
  EXPECT_TRUE(BatchTrue(q_out.Rotation().IsApprox(RotationZ(0.5F).Rotation())));
}
