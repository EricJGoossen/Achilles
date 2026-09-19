#include <gtest/gtest.h>

#include "algorithms/conventions.hpp"
#include "algorithms/pi/pi_ops.hpp"
#include "util/simd_ops.hpp"

// IntegratePositionOp wires S*qd (lifting a joint's generalized velocity
// into a real spatial twist -- the same lift PropagateVelocityOp performs
// in aba_ops.cpp) into Transform::Exp/operator* to advance a pose. Unlike
// IntegrateVelocityOp (algorithms_vi_vi_ops.cpp), this is not a vector
// accumulation: SE(3) doesn't compose additively, so most tests here
// confirm composition (`*`) is used, not addition -- right operand order,
// right frame (composed on the RIGHT of the existing pose, matching
// x_tree * x_joint's own local-frame convention in aba_ops.cpp) -- and one
// test exists specifically to catch a regression back to naive vector
// addition of the underlying coordinates, which silently gives the wrong
// answer for any joint with more than one active rotational DOF (see this
// file's own header comment history / aba_data.hpp's kJointPosition
// comment for why).
//
// Every domain type here is instantiated at achilles::algorithms'
// MathematicalT (xsimd::batch<float>), the only scalar shape this Op is
// ever actually called with in production.

using namespace achilles::algorithms;
using namespace achilles::algorithms::pi;

namespace {

using B = MathematicalT;

::testing::AssertionResult BatchTrue(const auto& mask) {
  if (achilles::util::AllTrue(mask)) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << "not all lanes true";
}

// Row 2 (yaw / angular-Z) driven by generalized-coordinate slot 0 -- the
// same single-DOF revolute joint tests/support/aba_reference.hpp's
// RevoluteZSubspace describes, kept local here rather than pulled in from
// that support header since this file has no other use for it.
Matrix6x6 RevoluteZSubspace() {
  Matrix6x6 s = Matrix6x6::Zero();
  s(2, 0) = B(1.0F);
  return s;
}

// Row 3 (linear-X, per Dual's Roll/Pitch/Yaw/X/Y/Z row order) driven by
// generalized-coordinate slot 0 -- a single-DOF prismatic joint sliding
// along its own local X axis.
Matrix6x6 PrismaticXSubspace() {
  Matrix6x6 s = Matrix6x6::Zero();
  s(3, 0) = B(1.0F);
  return s;
}

// Rows 0 and 1 (roll/pitch, angular-X and angular-Y) driven by generalized-
// coordinate slots 0 and 1 -- a 2-DOF universal joint, used to prove S
// combines more than one simultaneously active column into a single twist
// rather than only ever handling one nonzero generalized coordinate at a
// time.
Matrix6x6 UniversalXYSubspace() {
  Matrix6x6 s = Matrix6x6::Zero();
  s(0, 0) = B(1.0F);
  s(1, 1) = B(1.0F);
  return s;
}

}  // namespace

// qd == 0 means the incremental Transform::Exp is Exp(0) == Identity, so
// composing it on must leave the existing pose exactly unchanged --
// regardless of dt -- the same "nothing to add" guarantee
// IntegrateVelocityOpTest.ZeroAccelerationLeavesVelocityUnchanged proves
// for velocity, but via composition with Identity rather than addition of
// zero.
TEST(IntegratePositionOpTest, ZeroVelocityLeavesPositionUnchanged) {
  Transform x_initial(
      Vector3(B(1.0F), B(2.0F), B(3.0F)),
      Quaternion::Exp(Vector3(B(0.1F), B(0.2F), B(0.3F)))
  );
  Transform x_out = x_initial;

  IntegratePositionOp op(0.5F);
  op(RevoluteZSubspace(), Velocity::Zero(), &x_out);

  EXPECT_TRUE(BatchTrue(x_out.Translation().IsApprox(x_initial.Translation()))
  );
  EXPECT_TRUE(BatchTrue(x_out.Rotation().IsApprox(x_initial.Rotation())));
}

// dt == 0 means no time has passed -- the pose must survive unchanged no
// matter how large the velocity is, distinct from the zero-velocity case
// above (exercises dt's own zero, not qd's).
TEST(IntegratePositionOpTest, ZeroDtLeavesPositionUnchanged) {
  Transform x_initial(
      Vector3(B(1.0F), B(2.0F), B(3.0F)), Quaternion::Identity()
  );
  Transform x_out = x_initial;
  Velocity qd(Vector3(B(10.0F), B(-5.0F), B(2.0F)), Vector3::Zero());

  IntegratePositionOp op(0.0F);
  op(RevoluteZSubspace(), qd, &x_out);

  EXPECT_TRUE(BatchTrue(x_out.Translation().IsApprox(x_initial.Translation()))
  );
  EXPECT_TRUE(BatchTrue(x_out.Rotation().IsApprox(x_initial.Rotation())));
}

// Nonzero joint motion, starting from Identity: x_out recomputed
// independently via Transform::Exp/operator* (each already unit-tested on
// its own in domain_spatial_transform.cpp) rather than re-typing
// IntegratePositionOp's own expression.
TEST(IntegratePositionOpTest, SingleAxisRotationMatchesIndependentRecomputation) {
  Matrix6x6 s = RevoluteZSubspace();
  Vector6 qd_coords(B(0.4F), B(0.0F), B(0.0F), B(0.0F), B(0.0F), B(0.0F));
  Velocity qd(qd_coords);
  B dt(0.5F);

  Transform x_out = Transform::Identity();
  IntegratePositionOp op(0.5F);
  op(s, qd, &x_out);

  Velocity qd_spatial(s * qd_coords);
  Transform expected = Transform::Identity() * Transform::Exp(qd_spatial * dt);

  EXPECT_TRUE(BatchTrue(x_out.Translation().IsApprox(expected.Translation())));
  EXPECT_TRUE(BatchTrue(x_out.Rotation().IsApprox(expected.Rotation())));
}

// Starting from a non-identity pose: the increment must be composed onto
// it (x_initial * Exp(...)) rather than the Op overwriting x_out with just
// the fresh increment -- the position-integration counterpart of
// IntegrateVelocityOpTest.AccumulatesScaledAccelerationOntoExistingVelocity,
// with composition standing in for accumulation since a pose isn't a
// vector space.
TEST(IntegratePositionOpTest, ComposesOntoExistingPoseRatherThanOverwriting) {
  Matrix6x6 s = RevoluteZSubspace();
  Transform x_initial(
      Vector3(B(1.0F), B(0.0F), B(0.0F)),
      Quaternion::Exp(Vector3(B(0.0F), B(0.0F), B(0.3F)))
  );
  Vector6 qd_coords(B(0.4F), B(0.0F), B(0.0F), B(0.0F), B(0.0F), B(0.0F));
  Velocity qd(qd_coords);
  B dt(0.5F);

  Transform x_out = x_initial;
  IntegratePositionOp op(0.5F);
  op(s, qd, &x_out);

  Velocity qd_spatial(s * qd_coords);
  Transform expected = x_initial * Transform::Exp(qd_spatial * dt);

  EXPECT_TRUE(BatchTrue(x_out.Translation().IsApprox(expected.Translation())));
  EXPECT_TRUE(BatchTrue(x_out.Rotation().IsApprox(expected.Rotation())));
  // And, separately: the result must actually have moved from x_initial --
  // proving this isn't accidentally passing by leaving x_out untouched.
  EXPECT_FALSE(achilles::util::AllTrue(
      x_out.Rotation().IsApprox(x_initial.Rotation())
  ));
}

// The regression this file exists to catch: two sequential steps that
// each rotate about a DIFFERENT axis of a full 6-DOF joint (S = Identity)
// must compose the way finite rotations actually compose (order-
// dependent, via quaternion multiplication), not the way their
// coordinates would if the integrator naively summed q += qd*dt as flat
// vectors. Composing Exp(X-twist) then Exp(Y-twist) is NOT the same pose
// as Exp(X-twist + Y-twist) in one shot -- rotations about different axes
// don't commute -- so this test's expected value is built the same
// step-by-step way the Op itself must operate, and is checked against the
// wrong (single combined Exp of the summed coordinates) answer to prove
// the two actually differ for this input, not just that they happen to
// coincide.
TEST(IntegratePositionOpTest, SequentialStepsAboutDifferentAxesComposeNotAdd) {
  Matrix6x6 identity = Matrix6x6::Identity();
  B dt(1.0F);
  Vector3 x_axis_turn(B(1.5F), B(0.0F), B(0.0F));
  Vector3 y_axis_turn(B(0.0F), B(1.5F), B(0.0F));

  Transform x_out = Transform::Identity();
  IntegratePositionOp op(1.0F);

  op(identity, Velocity(x_axis_turn, Vector3::Zero()), &x_out);
  op(identity, Velocity(y_axis_turn, Vector3::Zero()), &x_out);

  Transform correctly_composed = Transform::Identity() *
                                  Transform::Exp(Velocity(x_axis_turn, Vector3::Zero()) * dt) *
                                  Transform::Exp(Velocity(y_axis_turn, Vector3::Zero()) * dt);
  Transform naively_summed = Transform::Exp(
      Velocity(x_axis_turn + y_axis_turn, Vector3::Zero()) * dt
  );

  EXPECT_TRUE(BatchTrue(x_out.Rotation().IsApprox(correctly_composed.Rotation())
  ));
  EXPECT_FALSE(achilles::util::AllTrue(
      x_out.Rotation().IsApprox(naively_summed.Rotation())
  ));
}

// A prismatic joint (pure translation, no rotation involved anywhere):
// Transform::Exp's rotation half degenerates to Identity, so this
// exercises the Linear() half of Exp/composition on its own, distinct
// from every rotation-only test above.
TEST(IntegratePositionOpTest, PureTranslationMatchesIndependentRecomputation) {
  Matrix6x6 s = PrismaticXSubspace();
  Vector6 qd_coords(B(2.0F), B(0.0F), B(0.0F), B(0.0F), B(0.0F), B(0.0F));
  Velocity qd(qd_coords);
  B dt(0.5F);

  Transform x_out = Transform::Identity();
  IntegratePositionOp op(0.5F);
  op(s, qd, &x_out);

  Velocity qd_spatial(s * qd_coords);
  Transform expected = Transform::Identity() * Transform::Exp(qd_spatial * dt);

  EXPECT_TRUE(BatchTrue(x_out.Translation().IsApprox(expected.Translation())));
  EXPECT_TRUE(BatchTrue(x_out.Rotation().IsApprox(expected.Rotation())));
  // And, concretely: a 2.0 rad/s slide along local X for 0.5s must move
  // exactly 1.0 along X, not merely "some nonzero amount".
  EXPECT_TRUE(BatchTrue(
      x_out.Translation().IsApprox(Vector3(B(1.0F), B(0.0F), B(0.0F)))
  ));
}

// A prismatic increment composed onto an already-rotated pose must land
// where the joint's OWN (rotated) local X axis points in world space, not
// where world-frame X points -- proving composition rotates the
// increment's translation by the existing orientation (Transform::
// operator*'s `rotation_.Rotate(other.translation_) + translation_`)
// rather than naively adding translations across two different frames.
TEST(
    IntegratePositionOpTest, TranslationIncrementIsRotatedIntoExistingOrientation
) {
  Matrix6x6 s = PrismaticXSubspace();
  // A 90-degree yaw: local X now points along world -Y... concretely,
  // Quaternion::Exp(Vector3(0,0,pi/2)) rotates world X onto world Y (a
  // fact this test only depends on, not re-derives -- rotation semantics
  // are domain_math_quaternion.cpp's job).
  constexpr float kHalfPi = 1.5707963267948966F;
  Transform x_initial(Vector3::Zero(), Quaternion::Exp(Vector3(B(0.0F), B(0.0F), B(kHalfPi))));
  Vector6 qd_coords(B(2.0F), B(0.0F), B(0.0F), B(0.0F), B(0.0F), B(0.0F));
  Velocity qd(qd_coords);
  B dt(0.5F);

  Transform x_out = x_initial;
  IntegratePositionOp op(0.5F);
  op(s, qd, &x_out);

  Velocity qd_spatial(s * qd_coords);
  Transform expected = x_initial * Transform::Exp(qd_spatial * dt);
  Transform wrong_naive_world_frame_add(
      x_initial.Translation() + Transform::Exp(qd_spatial * dt).Translation(),
      x_initial.Rotation()
  );

  EXPECT_TRUE(BatchTrue(x_out.Translation().IsApprox(expected.Translation())));
  EXPECT_FALSE(achilles::util::AllTrue(x_out.Translation().IsApprox(
      wrong_naive_world_frame_add.Translation()
  )));
}

// A 2-DOF joint with BOTH of its subspace columns active in the same call
// (not across separate Steps the way
// SequentialStepsAboutDifferentAxesComposeNotAdd exercises it): S must
// combine both generalized-velocity components into one twist (S*qd) and
// exponentiate that single combined twist once, matching how
// PropagateVelocityOp's own `S * qd` lift behaves for a real multi-DOF
// joint in aba_ops.cpp.
TEST(
    IntegratePositionOpTest, MultipleSimultaneousActiveDOFsCombineIntoOneTwist
) {
  Matrix6x6 s = UniversalXYSubspace();
  Vector6 qd_coords(B(0.3F), B(0.6F), B(0.0F), B(0.0F), B(0.0F), B(0.0F));
  Velocity qd(qd_coords);
  B dt(0.5F);

  Transform x_out = Transform::Identity();
  IntegratePositionOp op(0.5F);
  op(s, qd, &x_out);

  Velocity qd_spatial(s * qd_coords);
  Transform expected = Transform::Identity() * Transform::Exp(qd_spatial * dt);

  EXPECT_TRUE(BatchTrue(x_out.Rotation().IsApprox(expected.Rotation())));

  // The single-combined-twist result must differ from sequentially
  // exponentiating each column's own contribution as two separate
  // composed steps -- Exp(w1 + w2) != Exp(w1) * Exp(w2) in general, so a
  // regression that accidentally split this into two Exp calls (instead
  // of combining via S*qd first) would silently drift onto the wrong
  // pose.
  Vector6 x_only_coords(B(0.3F), B(0.0F), B(0.0F), B(0.0F), B(0.0F), B(0.0F));
  Vector6 y_only_coords(B(0.0F), B(0.6F), B(0.0F), B(0.0F), B(0.0F), B(0.0F));
  Transform sequential = Transform::Identity() *
                         Transform::Exp(Velocity(s * x_only_coords) * dt) *
                         Transform::Exp(Velocity(s * y_only_coords) * dt);
  EXPECT_FALSE(
      achilles::util::AllTrue(x_out.Rotation().IsApprox(sequential.Rotation()))
  );
}

// Integrating forward by a twist for dt and then backward by the exact
// negation of that same twist for the same dt must land back on the
// original pose exactly -- Exp(w*dt) and Exp(-w*dt) are inverses of each
// other along the same one-parameter subgroup (unlike the
// different-axes case above, a twist and its own negation always
// commute), so composing both in sequence must cancel regardless of how
// large w or dt is.
TEST(IntegratePositionOpTest, ForwardThenBackwardVelocityReturnsToOriginalPose) {
  Matrix6x6 s = RevoluteZSubspace();
  Transform x_initial(
      Vector3(B(1.0F), B(2.0F), B(3.0F)),
      Quaternion::Exp(Vector3(B(0.1F), B(0.0F), B(0.0F)))
  );
  Vector6 qd_coords(B(0.7F), B(0.0F), B(0.0F), B(0.0F), B(0.0F), B(0.0F));

  Transform x_out = x_initial;
  IntegratePositionOp op(0.25F);
  op(s, Velocity(qd_coords), &x_out);
  op(s, Velocity(qd_coords * B(-1.0F)), &x_out);

  EXPECT_TRUE(BatchTrue(x_out.Translation().IsApprox(x_initial.Translation())));
  EXPECT_TRUE(BatchTrue(x_out.Rotation().IsApprox(x_initial.Rotation())));
}
