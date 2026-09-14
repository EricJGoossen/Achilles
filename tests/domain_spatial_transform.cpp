#include <gtest/gtest.h>

#include "domain/math/matrix.hpp"
#include "domain/math/quaternion.hpp"
#include "domain/math/vector3.hpp"
#include "domain/spatial/dual.hpp"
#include "domain/spatial/inertia.hpp"
#include "domain/spatial/transform.hpp"

using achilles::domain::math::Matrix6x6;
using achilles::domain::math::Quaternion;
using achilles::domain::math::Vector3;
using achilles::domain::spatial::Inertia;
using achilles::domain::spatial::SpatialAcceleration;
using achilles::domain::spatial::SpatialForce;
using achilles::domain::spatial::SpatialVelocity;
using achilles::domain::spatial::Transform;

namespace {

constexpr float kPi = 3.14159265358979323846F;

// Transform has no IsApprox/== of its own -- comparing two Transforms
// means comparing their Translation() and Rotation() separately.
::testing::AssertionResult TransformNear(
    const Transform<float>& a, const Transform<float>& b, float epsilon = 1e-4F
) {
  if (a.Translation().IsApprox(b.Translation(), epsilon) &&
      a.Rotation().IsApprox(b.Rotation(), epsilon)) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure()
         << "translations/rotations differ by more than " << epsilon;
}

// A nontrivial, arbitrary transform -- nonzero translation and a rotation
// that isn't a multiple of a right angle -- used everywhere a "generic"
// transform is needed so tests don't accidentally pass only because
// something canceled out at a special-cased angle.
Transform<float> ArbitraryTransform() {
  return {
      Vector3<float>(1.0F, -2.0F, 0.5F),
      Quaternion<float>::Exp(Vector3<float>(0.3F, -0.6F, 0.2F))
  };
}

}  // namespace

// Transform() zero-initializes translation_ (a zero translation is a
// perfectly meaningful default) and default-constructs rotation_, which
// -- since Quaternion's own default constructor is the identity rotation,
// not the zero quaternion -- makes a default-constructed Transform the
// identity transform.
TEST(TransformConstruction, DefaultIsIdentity) {
  Transform<float> t;
  EXPECT_TRUE(t.Translation().IsZero());
  EXPECT_TRUE(t.Rotation().IsApprox(Quaternion<float>::Identity()));
}

TEST(TransformConstruction, FromTranslationAndRotation) {
  Vector3<float> translation(1.0F, 2.0F, 3.0F);
  Quaternion<float> rotation =
      Quaternion<float>::Exp(Vector3<float>(0.1F, 0.2F, 0.3F));
  Transform<float> t(translation, rotation);
  EXPECT_TRUE(t.Translation().IsApprox(translation));
  EXPECT_TRUE(t.Rotation().IsApprox(rotation));
}

TEST(TransformConstruction, FromSevenComponents) {
  Transform<float> t(1.0F, 2.0F, 3.0F, 1.0F, 0.0F, 0.0F, 0.0F);
  EXPECT_TRUE(t.Translation().IsApprox(Vector3<float>(1.0F, 2.0F, 3.0F)));
  EXPECT_TRUE(t.Rotation().IsApprox(Quaternion<float>::Identity()));
}

TEST(TransformStaticConstructors, IdentityAndSetIdentity) {
  Transform<float> id = Transform<float>::Identity();
  EXPECT_TRUE(id.Translation().IsZero());
  EXPECT_TRUE(id.Rotation().IsApprox(Quaternion<float>::Identity()));

  Transform<float> t = ArbitraryTransform();
  t.SetIdentity();
  EXPECT_TRUE(TransformNear(t, Transform<float>::Identity()));
}

// Exp(v) is documented as {v.Linear(), Quaternion::Exp(v.Angular())} --
// checked directly against those two independently-testable pieces
// (Quaternion::Exp itself is covered in domain_math_quaternion.cpp).
TEST(TransformStaticConstructors, ExpMatchesLinearAndQuaternionExp) {
  SpatialVelocity<float> v(
      Vector3<float>(0.0F, 0.0F, kPi / 2.0F), Vector3<float>(1.0F, 2.0F, 3.0F)
  );
  Transform<float> t = Transform<float>::Exp(v);
  EXPECT_TRUE(t.Translation().IsApprox(v.Linear()));
  EXPECT_TRUE(t.Rotation().IsApprox(Quaternion<float>::Exp(v.Angular())));
}

TEST(TransformAccess, TranslationRotationAndToTuple) {
  Vector3<float> translation(1.0F, 2.0F, 3.0F);
  Quaternion<float> rotation =
      Quaternion<float>::Exp(Vector3<float>(0.1F, 0.2F, 0.3F));
  Transform<float> t(translation, rotation);

  auto [tt, tr] = t.ToTuple();
  EXPECT_TRUE(tt.IsApprox(translation));
  EXPECT_TRUE(tr.IsApprox(rotation));
}

// operator*: composing with Identity on either side is a no-op.
TEST(TransformComposition, IdentityIsNoOp) {
  Transform<float> t = ArbitraryTransform();
  EXPECT_TRUE(TransformNear(t * Transform<float>::Identity(), t));
  EXPECT_TRUE(TransformNear(Transform<float>::Identity() * t, t));
}

TEST(TransformComposition, CompoundMultiplyMatchesBinary) {
  Transform<float> a = ArbitraryTransform();
  Transform<float> b(
      Vector3<float>(0.5F, 0.5F, -1.0F),
      Quaternion<float>::Exp(Vector3<float>(0.0F, 0.0F, kPi / 3.0F))
  );

  Transform<float> expected = a * b;
  Transform<float> in_place = a;
  in_place *= b;
  EXPECT_TRUE(TransformNear(in_place, expected));
}

// Inverse: composed with the original transform (either order) cancels to
// Identity -- the standard rigid-transform inverse property.
TEST(TransformInverse, ComposedWithOriginalIsIdentity) {
  Transform<float> t = ArbitraryTransform();
  EXPECT_TRUE(TransformNear(t * t.Inverse(), Transform<float>::Identity()));
  EXPECT_TRUE(TransformNear(t.Inverse() * t, Transform<float>::Identity()));
}

TEST(TransformInverse, InPlaceMatchesByValue) {
  Transform<float> t = ArbitraryTransform();
  Transform<float> expected = t.Inverse();

  Transform<float> in_place = t;
  in_place.InverseInPlace();
  EXPECT_TRUE(TransformNear(in_place, expected));
}

// Apply(Matrix6x6): Identity's congruence transform is a no-op (X = [[I,
// 0], [0, I]], so X*m*X^T == m for any m) -- avoids needing to
// hand-derive the sandwich formula for a nontrivial transform.
TEST(TransformApplyMatrix, IdentityIsNoOp) {
  Matrix6x6<float> m =
      Inertia<float>(
          2.0F, Vector3<float>::Zero(), 2.0F, 3.0F, 4.0F, 0.0F, 0.0F, 0.0F
      )
          .AsMatrix();
  EXPECT_TRUE(Transform<float>::Identity().Apply(m).IsApprox(m));
}

// Apply(SpatialVelocity/Acceleration/Force): Identity is a no-op, and --
// the property that actually exercises the math -- applying a composed
// transform matches applying each transform in turn. This holds
// algebraically for all three (rotate, then translate the frame origin
// into the rotated result) regardless of the specific values chosen, so
// it's checked with arbitrary transforms and an arbitrary spatial vector
// rather than a hand-derived numeric expectation.
TEST(TransformApplySpatialVelocity, IdentityIsNoOp) {
  SpatialVelocity<float> v(
      Vector3<float>(0.1F, -0.2F, 0.3F), Vector3<float>(1.0F, 2.0F, 3.0F)
  );
  EXPECT_TRUE(Transform<float>::Identity().Apply(v).IsApprox(v));
}

TEST(TransformApplySpatialVelocity, ComposedTransformMatchesSequentialApply) {
  Transform<float> t1 = ArbitraryTransform();
  Transform<float> t2(
      Vector3<float>(0.2F, 0.4F, -0.3F),
      Quaternion<float>::Exp(Vector3<float>(0.4F, 0.1F, -0.2F))
  );
  SpatialVelocity<float> v(
      Vector3<float>(0.1F, -0.2F, 0.3F), Vector3<float>(1.0F, 2.0F, 3.0F)
  );

  SpatialVelocity<float> composed = (t1 * t2).Apply(v);
  SpatialVelocity<float> sequential = t1.Apply(t2.Apply(v));
  EXPECT_TRUE(composed.IsApprox(sequential, 1e-4F));
}

TEST(
    TransformApplySpatialAcceleration, ComposedTransformMatchesSequentialApply
) {
  Transform<float> t1 = ArbitraryTransform();
  Transform<float> t2(
      Vector3<float>(0.2F, 0.4F, -0.3F),
      Quaternion<float>::Exp(Vector3<float>(0.4F, 0.1F, -0.2F))
  );
  SpatialAcceleration<float> a(
      Vector3<float>(0.1F, -0.2F, 0.3F), Vector3<float>(1.0F, 2.0F, 3.0F)
  );

  SpatialAcceleration<float> composed = (t1 * t2).Apply(a);
  SpatialAcceleration<float> sequential = t1.Apply(t2.Apply(a));
  EXPECT_TRUE(composed.IsApprox(sequential, 1e-4F));
}

TEST(TransformApplySpatialForce, ComposedTransformMatchesSequentialApply) {
  Transform<float> t1 = ArbitraryTransform();
  Transform<float> t2(
      Vector3<float>(0.2F, 0.4F, -0.3F),
      Quaternion<float>::Exp(Vector3<float>(0.4F, 0.1F, -0.2F))
  );
  SpatialForce<float> f(
      Vector3<float>(0.1F, -0.2F, 0.3F), Vector3<float>(1.0F, 2.0F, 3.0F)
  );

  SpatialForce<float> composed = (t1 * t2).Apply(f);
  SpatialForce<float> sequential = t1.Apply(t2.Apply(f));
  EXPECT_TRUE(composed.IsApprox(sequential, 1e-4F));
}

// Apply(Inertia) and Apply(InertiaOperator) are two independently-written
// formulas (a specialized sparse update vs. the generic Matrix6x6
// congruence transform via AsMatrix()) that must agree on the same
// physical rigid body -- a real bug in either one would very likely break
// this cross-check.
TEST(TransformApplyInertia, MatchesMatrixAndOperatorOverloads) {
  Inertia<float> i(
      2.0F, Vector3<float>::Zero(), 2.0F, 3.0F, 4.0F, 0.0F, 0.0F, 0.0F
  );
  Transform<float> t = ArbitraryTransform();

  Matrix6x6<float> via_sparse = t.Apply(i).AsMatrix();
  Matrix6x6<float> via_matrix = t.Apply(i.AsMatrix());
  Matrix6x6<float> via_operator = t.Apply(i.AsArticulated()).AsMatrix();

  EXPECT_TRUE(via_sparse.IsApprox(via_matrix, 1e-3F));
  EXPECT_TRUE(via_sparse.IsApprox(via_operator, 1e-3F));
}

TEST(TransformApplyInertia, IdentityIsNoOp) {
  Inertia<float> i(
      2.0F, Vector3<float>::Zero(), 2.0F, 3.0F, 4.0F, 0.0F, 0.0F, 0.0F
  );
  EXPECT_TRUE(Transform<float>::Identity().Apply(i).IsApprox(i));
}
