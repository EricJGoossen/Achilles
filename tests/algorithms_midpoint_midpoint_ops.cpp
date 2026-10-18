#include <gtest/gtest.h>

#include <cmath>

#include "algorithms/conventions.hpp"
#include "algorithms/midpoint/midpoint_ops.hpp"
#include "util/simd_ops.hpp"

// Implicit midpoint's four per-row updates, called directly:
//   BeginStepOp          (q, qd)         -> (q_n, qd_n, qd_next = qd_n)
//   MidpointTrialOp      (S, q_n, qd_n, qd_next) -> trial (q, qd)
//   UpdateNextVelocityOp (qd_n, qdd)     -> qd_next = qd_n + qdd dt
//   CommitOp             (S, q_n, qd_n, qd_next) -> committed (q, qd)
// algorithms_midpoint_midpoint_step.cpp covers them through the real
// View/traversal; algorithms_integrators.cpp covers the whole tick.
//
// All four overwrite their outputs: none of their incoming output values
// is meaningful (see each Op's comment), so each test seeds the outputs
// with junk to prove the Op doesn't read them.
//
// Pose expectations use the closed form for a revolute-Z joint (rotation by
// theta about Z is (cos(theta/2), 0, 0, sin(theta/2))) rather than
// recomputing Transform::Exp.

using namespace achilles::algorithms;
using namespace achilles::algorithms::midpoint;

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

// An obviously-wrong value to seed an output with.
Transform JunkPose() {
  return {
      Vector3(B(9.0F), B(9.0F), B(9.0F)),
      Quaternion(B(0.0F), B(1.0F), B(0.0F), B(0.0F))
  };
}

}  // namespace

// Snapshots the tick's start and seeds the fixed-point estimate with qd_n.
TEST(MidpointBeginStepOpTest, SnapshotsStartStateAndSeedsTheEstimate) {
  Transform q_start_out = JunkPose();
  Velocity qd_start_out = Slot0(-9.0F);
  Velocity qd_next_out = Slot0(-9.0F);

  BeginStepOp op;
  op(RotationZ(0.3F), Slot0(1.5F), &q_start_out, &qd_start_out, &qd_next_out);

  EXPECT_TRUE(
      BatchTrue(q_start_out.Rotation().IsApprox(RotationZ(0.3F).Rotation()))
  );
  EXPECT_TRUE(BatchTrue(q_start_out.Translation().IsZero()));
  EXPECT_TRUE(BatchTrue(qd_start_out.IsApprox(Slot0(1.5F))));
  EXPECT_TRUE(BatchTrue(qd_next_out.IsApprox(Slot0(1.5F))));
}

// The trial is evaluated at qd_mid = (qd_n + qd_next)/2, and q_n advanced
// by qd_mid over HALF the dt -- the midpoint in time, not the endpoint.
TEST(MidpointTrialOpTest, WritesTheHalfDtMidpointOfStartAndEstimate) {
  Transform q_out = JunkPose();
  Velocity qd_out = Slot0(-9.0F);

  MidpointTrialOp op(0.5F);
  op(RevoluteZSubspace(),
     RotationZ(0.1F),
     Slot0(1.0F),
     Slot0(3.0F),
     &q_out,
     &qd_out);

  // qd_mid = 2; q = 0.1 + 2 * 0.25.
  EXPECT_TRUE(BatchTrue(qd_out.IsApprox(Slot0(2.0F))));
  EXPECT_TRUE(BatchTrue(q_out.Rotation().IsApprox(RotationZ(0.6F).Rotation())));
}

// qd_next = qd_n + qdd * dt -- from the tick's START velocity, not by
// accumulating onto the previous estimate (which would compound across
// fixed-point iterations).
TEST(UpdateNextVelocityOpTest, EstimatesFromStartVelocityNotPreviousEstimate) {
  Velocity qd_next_out = Slot0(100.0F);

  UpdateNextVelocityOp op(0.5F);
  op(Slot0(1.0F), Slot0Acceleration(4.0F), &qd_next_out);

  EXPECT_TRUE(BatchTrue(qd_next_out.IsApprox(Slot0(3.0F))));
}

// The commit advances q_n by qd_mid over the FULL dt (unlike the trial's
// half dt) and commits the estimate as the new velocity.
TEST(MidpointCommitOpTest, CommitsFullDtPositionAndEstimatedVelocity) {
  Transform q_out = JunkPose();
  Velocity qd_out = Slot0(-9.0F);

  CommitOp op(0.5F);
  op(RevoluteZSubspace(),
     RotationZ(0.1F),
     Slot0(1.0F),
     Slot0(3.0F),
     &q_out,
     &qd_out);

  // qd_mid = 2; q = 0.1 + 2 * 0.5.
  EXPECT_TRUE(BatchTrue(q_out.Rotation().IsApprox(RotationZ(1.1F).Rotation())));
  EXPECT_TRUE(BatchTrue(qd_out.IsApprox(Slot0(3.0F))));
}

// The committed pose accumulates across ticks, so it's renormalized: a
// start rotation that has drifted off unit length comes back unit-length.
TEST(MidpointCommitOpTest, RenormalizesTheCommittedRotation) {
  Transform q_start(
      Vector3::Zero(), Quaternion(B(1.01F), B(0.0F), B(0.0F), B(0.0F))
  );
  Transform q_out;
  Velocity qd_out;

  CommitOp op(0.5F);
  op(RevoluteZSubspace(), q_start, Slot0(1.0F), Slot0(1.0F), &q_out, &qd_out);

  EXPECT_TRUE(
      BatchTrue(xsimd::abs(q_out.Rotation().Norm() - B(1.0F)) < B(1e-6F))
  );
  EXPECT_TRUE(BatchTrue(q_out.Rotation().IsApprox(RotationZ(0.5F).Rotation())));
}
