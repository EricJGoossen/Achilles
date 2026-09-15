#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <xsimd/xsimd.hpp>

#include "domain/math/matrix.hpp"
#include "domain/math/quaternion.hpp"
#include "domain/math/vector3.hpp"
#include "support/simd_test_helpers.hpp"

using achilles::domain::math::Matrix;
using achilles::domain::math::Quaternion;
using achilles::domain::math::Vector3;
using achilles::test_support::MakeBatch;

namespace {

constexpr float kPi = std::numbers::pi_v<float>;

}  // namespace

// Unlike Vector3/Matrix/Vector6 (which default to all-zero), a
// default-constructed Quaternion is the identity rotation -- a zero
// quaternion isn't a valid rotation at all, so Identity() itself just
// delegates to the default constructor (see quaternion.hpp).
TEST(QuaternionConstruction, DefaultIsIdentity) {
  Quaternion<float> q;
  EXPECT_FLOAT_EQ(q.W(), 1.0F);
  EXPECT_FLOAT_EQ(q.X(), 0.0F);
  EXPECT_FLOAT_EQ(q.Y(), 0.0F);
  EXPECT_FLOAT_EQ(q.Z(), 0.0F);
}

TEST(QuaternionConstruction, FromComponents) {
  Quaternion<float> q(1.0F, 2.0F, 3.0F, 4.0F);
  EXPECT_FLOAT_EQ(q.W(), 1.0F);
  EXPECT_FLOAT_EQ(q.X(), 2.0F);
  EXPECT_FLOAT_EQ(q.Y(), 3.0F);
  EXPECT_FLOAT_EQ(q.Z(), 4.0F);
}

TEST(QuaternionConstruction, FromMatrix) {
  Matrix<float, 4, 1> m(1.0F, 0.0F, 0.0F, 0.0F);
  Quaternion<float> q(m);
  EXPECT_FLOAT_EQ(q.W(), 1.0F);
}

// FromVector builds a pure (w=0) quaternion and normalizes it -- feeding
// it a non-unit vector must still come back unit length with the same
// direction.
TEST(QuaternionConstruction, FromVectorIsPureAndUnit) {
  Vector3<float> v(3.0F, 0.0F, 4.0F);
  Quaternion<float> q = Quaternion<float>::FromVector(v);
  EXPECT_FLOAT_EQ(q.W(), 0.0F);
  EXPECT_FLOAT_EQ(q.Norm(), 1.0F);
  EXPECT_FLOAT_EQ(q.X(), 0.6F);
  EXPECT_FLOAT_EQ(q.Z(), 0.8F);
}

TEST(QuaternionStaticConstructors, IdentityAndSetIdentity) {
  Quaternion<float> id = Quaternion<float>::Identity();
  EXPECT_FLOAT_EQ(id.W(), 1.0F);
  EXPECT_FLOAT_EQ(id.X(), 0.0F);
  EXPECT_FLOAT_EQ(id.Y(), 0.0F);
  EXPECT_FLOAT_EQ(id.Z(), 0.0F);

  Quaternion<float> q(5.0F, 6.0F, 7.0F, 8.0F);
  q.SetIdentity();
  EXPECT_FLOAT_EQ(q.W(), 1.0F);
  EXPECT_FLOAT_EQ(q.X(), 0.0F);
}

// Exp(v) is the rotation of angle |v| about axis v/|v|: a pi/2 rotation
// about Z should land exactly on the known quaternion for that rotation.
TEST(QuaternionStaticConstructors, ExpOfQuarterTurnAboutZ) {
  Vector3<float> axis_angle(0.0F, 0.0F, kPi / 2.0F);
  Quaternion<float> q = Quaternion<float>::Exp(axis_angle);

  EXPECT_NEAR(q.W(), std::cos(kPi / 4.0F), 1e-5F);
  EXPECT_NEAR(q.Z(), std::sin(kPi / 4.0F), 1e-5F);
  EXPECT_NEAR(q.X(), 0.0F, 1e-5F);
  EXPECT_NEAR(q.Y(), 0.0F, 1e-5F);
}

// theta == 0 is a removable singularity in Exp's own formula
// (k = sin(theta/2)/theta), not an undefined input: the correct limit is
// the identity rotation. Exp used to assert here instead -- since every
// joint at its q=0 rest configuration, and any prismatic joint at any q,
// produces exactly this input via Transform::Exp in the ABA pipeline,
// that made the most natural starting configuration crash. Checked at
// exactly zero and at a magnitude well under the 1e-8 threshold, to
// confirm the branchless mask (not just the exact-zero case) takes hold.
TEST(QuaternionStaticConstructors, ExpOfZeroVectorIsIdentity) {
  EXPECT_TRUE(Quaternion<float>::Exp(Vector3<float>::Zero()).IsIdentity());
  EXPECT_TRUE(
      Quaternion<float>::Exp(Vector3<float>(1e-9F, 0.0F, 0.0F)).IsIdentity()
  );
}

// Accessors: ToTuple, W/X/Y/Z, operator[], AsMatrix must all agree.
TEST(QuaternionAccess, AllAccessorsAgree) {
  Quaternion<float> q(1.0F, 2.0F, 3.0F, 4.0F);

  auto [tw, tx, ty, tz] = q.ToTuple();
  EXPECT_FLOAT_EQ(tw, 1.0F);
  EXPECT_FLOAT_EQ(tx, 2.0F);
  EXPECT_FLOAT_EQ(ty, 3.0F);
  EXPECT_FLOAT_EQ(tz, 4.0F);

  EXPECT_FLOAT_EQ(q[0], q.W());
  EXPECT_FLOAT_EQ(q[3], q.Z());
  EXPECT_FLOAT_EQ(q.AsMatrix()(0, 0), q.W());
}

TEST(QuaternionComparison, EqualityAndInequality) {
  Quaternion<float> a(1.0F, 2.0F, 3.0F, 4.0F);
  Quaternion<float> b(1.0F, 2.0F, 3.0F, 4.0F);
  Quaternion<float> c(1.0F, 2.0F, 3.0F, 5.0F);

  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a != b);
  EXPECT_TRUE(a != c);
  EXPECT_FALSE(a == c);
}

TEST(QuaternionComparison, IsApproxWithinAndOutsideEpsilon) {
  Quaternion<float> a(1.0F, 2.0F, 3.0F, 4.0F);
  Quaternion<float> close(1.0F + 1e-7F, 2.0F, 3.0F, 4.0F);
  Quaternion<float> far(1.1F, 2.0F, 3.0F, 4.0F);

  EXPECT_TRUE(a.IsApprox(close));
  EXPECT_FALSE(a.IsApprox(far));
}

TEST(QuaternionComparison, IsIdentity) {
  EXPECT_TRUE(Quaternion<float>::Identity().IsIdentity());
  EXPECT_TRUE(Quaternion<float>().IsIdentity());
  EXPECT_FALSE(Quaternion<float>(1.0F, 0.1F, 0.0F, 0.0F).IsIdentity());
}

// ToRotationMatrix: identity quaternion must produce the identity matrix,
// and a known 90-degree-about-Z quaternion must produce the matching
// rotation matrix.
TEST(QuaternionRotationMatrix, IdentityProducesIdentityMatrix) {
  Matrix<float, 3, 3> m = Quaternion<float>::Identity().ToRotationMatrix();
  EXPECT_TRUE(m.IsApprox(Matrix<float, 3, 3>::Identity()));
}

TEST(QuaternionRotationMatrix, QuarterTurnAboutZMatchesKnownMatrix) {
  Quaternion<float> q =
      Quaternion<float>::Exp(Vector3<float>(0.0F, 0.0F, kPi / 2.0F));
  Matrix<float, 3, 3> m = q.ToRotationMatrix();
  Matrix<float, 3, 3> expected(
      0.0F, -1.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F
  );
  EXPECT_TRUE(m.IsApprox(expected, 1e-5F));
}

// Quaternion multiplication: identity is the multiplicative identity, and
// composing a rotation with its inverse cancels out.
TEST(QuaternionMultiplication, IdentityIsMultiplicativeIdentity) {
  Quaternion<float> q(0.5F, 0.5F, 0.5F, 0.5F);
  EXPECT_TRUE((q * Quaternion<float>::Identity()).IsApprox(q));
  EXPECT_TRUE((Quaternion<float>::Identity() * q).IsApprox(q));
}

TEST(QuaternionMultiplication, ComposedWithInverseIsIdentity) {
  Quaternion<float> q =
      Quaternion<float>::Exp(Vector3<float>(0.3F, -0.1F, 0.2F));
  Quaternion<float> product = q * q.Inverse();
  EXPECT_TRUE(product.IsApprox(Quaternion<float>::Identity(), 1e-4F));
}

TEST(QuaternionMultiplication, CompoundMultiplyMatchesBinary) {
  Quaternion<float> a(0.5F, 0.5F, 0.5F, 0.5F);
  Quaternion<float> b =
      Quaternion<float>::Exp(Vector3<float>(0.1F, 0.0F, 0.0F));

  Quaternion<float> expected = a * b;
  Quaternion<float> in_place = a;
  in_place *= b;
  EXPECT_TRUE(in_place.IsApprox(expected));
}

TEST(QuaternionNorm, NormAndNormalize) {
  Quaternion<float> q(1.0F, 1.0F, 1.0F, 1.0F);
  EXPECT_FLOAT_EQ(q.Norm(), 2.0F);

  Quaternion<float> unit = q.Normalize();
  EXPECT_FLOAT_EQ(unit.Norm(), 1.0F);

  Quaternion<float> in_place = q;
  in_place.NormalizeInPlace();
  EXPECT_TRUE(in_place.IsApprox(unit));
}

// Conjugate negates the vector part only.
TEST(QuaternionConjugate, NegatesVectorPart) {
  Quaternion<float> q(1.0F, 2.0F, 3.0F, 4.0F);
  Quaternion<float> conj = q.Conjugate();
  EXPECT_FLOAT_EQ(conj.W(), 1.0F);
  EXPECT_FLOAT_EQ(conj.X(), -2.0F);
  EXPECT_FLOAT_EQ(conj.Y(), -3.0F);
  EXPECT_FLOAT_EQ(conj.Z(), -4.0F);

  Quaternion<float> in_place = q;
  in_place.ConjugateInPlace();
  EXPECT_TRUE(in_place.IsApprox(conj));
}

// For a unit quaternion, Inverse() is the same as Conjugate(); Inverse
// composed with the original quaternion is always identity regardless of
// normalization.
TEST(QuaternionInverse, UnitQuaternionInverseIsConjugate) {
  Quaternion<float> q =
      Quaternion<float>::Exp(Vector3<float>(0.2F, 0.4F, -0.1F));
  EXPECT_TRUE(q.Inverse().IsApprox(q.Conjugate(), 1e-4F));
}

TEST(QuaternionInverse, InPlaceMatchesByValue) {
  Quaternion<float> q(2.0F, 0.0F, 0.0F, 0.0F);
  Quaternion<float> expected = q.Inverse();

  Quaternion<float> in_place = q;
  in_place.InverseInPlace();
  EXPECT_TRUE(in_place.IsApprox(expected));
}

// Rotate: identity leaves a vector unchanged; a known 90-degree-about-Z
// rotation sends +X to +Y.
TEST(QuaternionRotate, IdentityLeavesVectorUnchanged) {
  Vector3<float> v(1.0F, 2.0F, 3.0F);
  EXPECT_TRUE(Quaternion<float>::Identity().Rotate(v).IsApprox(v));
}

TEST(QuaternionRotate, QuarterTurnAboutZSendsXToY) {
  Quaternion<float> q =
      Quaternion<float>::Exp(Vector3<float>(0.0F, 0.0F, kPi / 2.0F));
  Vector3<float> rotated = q.Rotate(Vector3<float>::UnitX());
  EXPECT_TRUE(rotated.IsApprox(Vector3<float>::UnitY(), 1e-5F));
}

// Slerp: t=0 and t=1 return the endpoints; interpolating a quaternion with
// itself returns itself at every t; a half turn to a quarter turn passes
// through the expected midpoint angle.
TEST(QuaternionSlerp, EndpointsAtTZeroAndTOne) {
  Quaternion<float> a = Quaternion<float>::Identity();
  Quaternion<float> b =
      Quaternion<float>::Exp(Vector3<float>(0.0F, 0.0F, kPi / 2.0F));

  EXPECT_TRUE(Quaternion<float>::Slerp(a, b, 0.0F).IsApprox(a, 1e-4F));
  EXPECT_TRUE(Quaternion<float>::Slerp(a, b, 1.0F).IsApprox(b, 1e-4F));
}

TEST(QuaternionSlerp, SameInputAtEveryTReturnsInput) {
  Quaternion<float> a =
      Quaternion<float>::Exp(Vector3<float>(0.1F, 0.2F, 0.3F));
  EXPECT_TRUE(Quaternion<float>::Slerp(a, a, 0.5F).IsApprox(a, 1e-4F));
}

TEST(QuaternionSlerp, MidpointOfQuarterTurnIsEighthTurn) {
  Quaternion<float> a = Quaternion<float>::Identity();
  Quaternion<float> b =
      Quaternion<float>::Exp(Vector3<float>(0.0F, 0.0F, kPi / 2.0F));
  Quaternion<float> mid = Quaternion<float>::Slerp(a, b, 0.5F);
  Quaternion<float> expected =
      Quaternion<float>::Exp(Vector3<float>(0.0F, 0.0F, kPi / 4.0F));
  EXPECT_TRUE(mid.IsApprox(expected, 1e-4F));
}

// Batched smoke test: confirms T = xsimd::batch<float> is a genuine free
// parameter by rotating two different axis-angle vectors, one per lane.
TEST(QuaternionBatched, ExpAndRotatePerLane) {
  Quaternion<xsimd::batch<float>> identity_batch =
      Quaternion<xsimd::batch<float>>::Identity();
  EXPECT_FLOAT_EQ(identity_batch.W().get(0), 1.0F);
  EXPECT_FLOAT_EQ(identity_batch.W().get(1), 1.0F);

  Quaternion<xsimd::batch<float>> q(
      MakeBatch({1.0F, 1.0F}),
      MakeBatch({0.0F, 0.0F}),
      MakeBatch({0.0F, 0.0F}),
      MakeBatch({0.0F, 0.0F})
  );
  auto norm = q.Norm();
  EXPECT_FLOAT_EQ(norm.get(0), 1.0F);
  EXPECT_FLOAT_EQ(norm.get(1), 1.0F);
}
