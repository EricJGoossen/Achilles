#include <gtest/gtest.h>

#include <cmath>

#include "algorithms/conventions.hpp"
#include "algorithms/verlet/verlet_ops.hpp"
#include "util/simd_ops.hpp"

// KickVelocityOp (qd += qdd * dt) and DriftPositionOp (q = q * Exp(S qd dt),
// renormalized) -- velocity Verlet's two per-row updates. Called directly
// here; algorithms_verlet_verlet_step.cpp covers them through the real
// View/traversal.
//
// Pose expectations use the closed form for a revolute-Z joint -- rotation
// by theta about Z is the quaternion (cos(theta/2), 0, 0, sin(theta/2)) --
// rather than recomputing Transform::Exp, so a wrong Exp argument (missing
// dt, wrong scale) can't cancel out.
//
// Every domain type is instantiated at MathematicalT, the only shape these
// Ops are called with in production; values are broadcast across lanes.

using namespace achilles::algorithms;
using namespace achilles::algorithms::verlet;

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

// Generalized velocity `rate` in slot 0.
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

}  // namespace

// The kick must add onto the incoming velocity (+=), not overwrite it --
// Verlet's second half-kick lands on top of the first (TESTING.md 9.2).
TEST(KickVelocityOpTest, AccumulatesScaledAccelerationOntoIncomingVelocity) {
  Velocity qd_out = Slot0(1.0F);

  KickVelocityOp op(0.25F);
  op(Slot0Acceleration(4.0F), &qd_out);

  EXPECT_TRUE(BatchTrue(qd_out.IsApprox(Slot0(2.0F))));
}

// dt == 0 contributes nothing, however large the acceleration.
TEST(KickVelocityOpTest, ZeroDtLeavesVelocityUnchanged) {
  Velocity qd_out = Slot0(1.0F);

  KickVelocityOp op(0.0F);
  op(Slot0Acceleration(100.0F), &qd_out);

  EXPECT_TRUE(BatchTrue(qd_out.IsApprox(Slot0(1.0F))));
}

// From identity, a drift is a rotation by qd * dt about the joint's axis.
TEST(DriftPositionOpTest, RotatesByVelocityTimesDtAboutTheJointAxis) {
  Transform q_out = Transform::Identity();

  DriftPositionOp op(0.5F);
  op(RevoluteZSubspace(), Slot0(0.8F), &q_out);

  EXPECT_TRUE(BatchTrue(q_out.Rotation().IsApprox(RotationZ(0.4F).Rotation())));
  EXPECT_TRUE(BatchTrue(q_out.Translation().IsZero()));
}

// The increment composes onto the incoming pose rather than replacing it:
// for one fixed axis, rotations add.
TEST(DriftPositionOpTest, ComposesOntoTheIncomingPose) {
  Transform q_out = RotationZ(0.3F);

  DriftPositionOp op(0.5F);
  op(RevoluteZSubspace(), Slot0(0.8F), &q_out);

  EXPECT_TRUE(BatchTrue(q_out.Rotation().IsApprox(RotationZ(0.7F).Rotation())));
}

// The committed pose is renormalized -- the drift accumulates across ticks,
// so an incoming rotation that has drifted off unit length must come back
// as a unit quaternion.
TEST(DriftPositionOpTest, RenormalizesTheRotation) {
  Transform q_out(
      Vector3::Zero(), Quaternion(B(1.01F), B(0.0F), B(0.0F), B(0.0F))
  );

  DriftPositionOp op(0.5F);
  op(RevoluteZSubspace(), Slot0(0.8F), &q_out);

  EXPECT_TRUE(
      BatchTrue(xsimd::abs(q_out.Rotation().Norm() - B(1.0F)) < B(1e-6F))
  );
  EXPECT_TRUE(BatchTrue(q_out.Rotation().IsApprox(RotationZ(0.4F).Rotation())));
}
