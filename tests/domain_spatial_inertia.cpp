#include <gtest/gtest.h>

#include "domain/math/matrix.hpp"
#include "domain/math/quaternion.hpp"
#include "domain/math/vector3.hpp"
#include "domain/spatial/dual.hpp"
#include "domain/spatial/inertia.hpp"
#include "domain/spatial/transform.hpp"
#include "support/mask_archetype.hpp"

using achilles::domain::math::Matrix3x3;
using achilles::domain::math::Matrix6x6;
using achilles::domain::math::Quaternion;
using achilles::domain::math::Vector3;
using achilles::domain::spatial::ArticulatedInertiaOperator;
using achilles::domain::spatial::Inertia;
using achilles::domain::spatial::InertiaOperator;
using achilles::domain::spatial::InvertedArticulatedInertiaOperator;
using achilles::domain::spatial::SpatialAcceleration;
using achilles::domain::spatial::SpatialForce;
using achilles::domain::spatial::SpatialMomentum;
using achilles::domain::spatial::SpatialVelocity;
using achilles::domain::spatial::Transform;
using achilles::test_support::MaskArchetype;

namespace {

// A simple, trivially physically-valid rigid body: mass 2 at its own
// center of mass (h=0), diagonal principal moments (2, 3, 4). Diagonal
// with h=0 means IsPhysicallyValid's eigenvalues are just (2,3,4)
// themselves -- PSD and the triangle inequality (2+3>=4) both hold
// without needing any eigenvalue computation to verify by hand.
Inertia<float> ValidInertia() {
  return {2.0F, Vector3<float>::Zero(), 2.0F, 3.0F, 4.0F, 0.0F, 0.0F, 0.0F};
}

// Same physical body, re-expressed at a different reference point (h !=
// 0). Built via Transform::Apply rather than hand-derived, since a
// translated valid inertia is always still valid (parallel axis theorem)
// -- verified independently in domain_spatial_transform.cpp
// (TransformApplyInertia) -- so this can't accidentally violate the
// physical-validity assert ValidInertia() itself only avoids by being
// diagonal.
Inertia<float> ShiftedInertia() {
  return Transform<float>(
             Vector3<float>(0.3F, -0.2F, 0.1F), Quaternion<float>::Identity()
  )
      .Apply(ValidInertia());
}

}  // namespace

// ===== Inertia<T> =====

TEST(InertiaConstruction, DefaultIsZero) {
  Inertia<float> i;
  EXPECT_TRUE(i.IsZero());
}

TEST(InertiaConstruction, FromEightComponents) {
  Inertia<float> i = ValidInertia();
  EXPECT_FLOAT_EQ(i.Mass(), 2.0F);
  EXPECT_TRUE(i.H().IsZero());
  EXPECT_FLOAT_EQ(i.Ixx(), 2.0F);
  EXPECT_FLOAT_EQ(i.Iyy(), 3.0F);
  EXPECT_FLOAT_EQ(i.Izz(), 4.0F);
  EXPECT_FLOAT_EQ(i.Ixy(), 0.0F);
  EXPECT_FLOAT_EQ(i.Ixz(), 0.0F);
  EXPECT_FLOAT_EQ(i.Iyz(), 0.0F);
}

// The (mass, h, Matrix3x3) constructor must agree with the 8-component
// one for the same physical inertia, and asserts the matrix it's given is
// symmetric (checked here only by construction, not violated).
TEST(InertiaConstruction, FromSymmetricMatrix3x3MatchesEightComponentForm) {
  Matrix3x3<float> i_com(2.0F, 0.0F, 0.0F, 0.0F, 3.0F, 0.0F, 0.0F, 0.0F, 4.0F);
  Inertia<float> from_matrix(2.0F, Vector3<float>::Zero(), i_com);
  EXPECT_TRUE(from_matrix.IsApprox(ValidInertia()));
}

// Constructing a physically unrealizable inertia (negative mass, or
// principal moments that violate the triangle inequality a real rigid
// body's moments always satisfy) must fail the constructor's validity
// assert rather than silently accept nonsense.
TEST(InertiaConstruction, RejectsNegativeMass) {
  EXPECT_DEATH(
      Inertia<float>(
          -1.0F, Vector3<float>::Zero(), 1.0F, 1.0F, 1.0F, 0.0F, 0.0F, 0.0F
      ),
      ""
  );
}

TEST(InertiaConstruction, RejectsTriangleInequalityViolation) {
  // Eigenvalues (with h=0) are just the diagonal: (0.1, 0.1, 10) fails
  // 0.1 + 0.1 >= 10.
  EXPECT_DEATH(
      Inertia<float>(
          1.0F, Vector3<float>::Zero(), 0.1F, 0.1F, 10.0F, 0.0F, 0.0F, 0.0F
      ),
      ""
  );
}

TEST(InertiaStaticConstructors, ZeroAndSetZero) {
  EXPECT_TRUE(Inertia<float>::Zero().IsZero());

  Inertia<float> i = ValidInertia();
  i.SetZero();

  EXPECT_TRUE(i.IsZero());
}

TEST(InertiaStaticConstructors, IdentityAndSetIdentity) {
  Inertia<float> id = Inertia<float>::Identity();
  EXPECT_FLOAT_EQ(id.Mass(), 1.0F);
  EXPECT_TRUE(id.H().IsZero());
  EXPECT_FLOAT_EQ(id.Ixx(), 1.0F);
  EXPECT_FLOAT_EQ(id.Iyy(), 1.0F);
  EXPECT_FLOAT_EQ(id.Izz(), 1.0F);

  Inertia<float> i = ValidInertia();
  i.SetIdentity();
  EXPECT_TRUE(i.IsApprox(Inertia<float>::Identity()));
}

// Access: ToTuple, Mass/H/Ixx.../AsMatrix/RotationalMatrix/AsArticulated
// must all agree with each other and with the constructor's arguments.
TEST(InertiaAccess, AllAccessorsAgree) {
  Inertia<float> i = ValidInertia();

  auto [mass, h, ixx, iyy, izz, ixy, ixz, iyz] = i.ToTuple();
  EXPECT_FLOAT_EQ(mass, i.Mass());
  EXPECT_TRUE(h.IsApprox(i.H()));
  EXPECT_FLOAT_EQ(ixx, i.Ixx());
  EXPECT_FLOAT_EQ(iyy, i.Iyy());
  EXPECT_FLOAT_EQ(izz, i.Izz());
  EXPECT_FLOAT_EQ(ixy, i.Ixy());
  EXPECT_FLOAT_EQ(ixz, i.Ixz());
  EXPECT_FLOAT_EQ(iyz, i.Iyz());

  Matrix3x3<float> rot = i.RotationalMatrix();
  EXPECT_FLOAT_EQ(rot(0, 0), i.Ixx());
  EXPECT_FLOAT_EQ(rot(1, 1), i.Iyy());
  EXPECT_FLOAT_EQ(rot(2, 2), i.Izz());

  // AsMatrix's bottom-right block is mass*Identity; top-left is
  // RotationalMatrix(); off-diagonal blocks are h's skew (zero here).
  Matrix6x6<float> m = i.AsMatrix();
  EXPECT_FLOAT_EQ(m(3, 3), i.Mass());
  EXPECT_TRUE((m.Submatrix<3, 3>(0, 0).IsApprox(rot)));

  EXPECT_TRUE(i.AsArticulated().AsMatrix().IsApprox(m));
}

TEST(InertiaComparison, EqualityAndInequality) {
  Inertia<float> a = ValidInertia();
  Inertia<float> b = a;
  Inertia<float> c = Inertia<float>::Identity();

  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a != b);
  EXPECT_TRUE(a != c);
  EXPECT_FALSE(a == c);
}

TEST(InertiaComparison, IsApproxAndIsZero) {
  Inertia<float> a = ValidInertia();
  Inertia<float> close = a * 1.0000001F;
  EXPECT_TRUE(a.IsApprox(close));

  EXPECT_TRUE(Inertia<float>::Zero().IsZero());
  EXPECT_FALSE(a.IsZero());
}

// Elementwise arithmetic: Inertia+Inertia, Inertia+InertiaOperator (and
// vice versa via InertiaOperator's own operator+), same for -, and their
// in-place forms. The mixed-type overloads are checked against composing
// through AsArticulated() first, since that's documented to be exactly
// what they do internally.
TEST(InertiaArithmetic, InertiaPlusInertia) {
  Inertia<float> a = ValidInertia();
  Inertia<float> sum = a + a;
  EXPECT_FLOAT_EQ(sum.Mass(), 4.0F);
  EXPECT_FLOAT_EQ(sum.Ixx(), 4.0F);

  Inertia<float> in_place = a;
  in_place += a;
  EXPECT_TRUE(in_place.IsApprox(sum));
}

TEST(InertiaArithmetic, InertiaMinusInertia) {
  Inertia<float> a = ValidInertia();
  Inertia<float> diff = (a + a) - a;
  EXPECT_TRUE(diff.IsApprox(a));

  Inertia<float> in_place = a + a;
  in_place -= a;
  EXPECT_TRUE(in_place.IsApprox(a));
}

TEST(InertiaArithmetic, InertiaPlusInertiaOperatorMatchesArticulatedForm) {
  Inertia<float> a = ValidInertia();
  InertiaOperator<float> expected = a.AsArticulated() + a.AsArticulated();
  EXPECT_TRUE((a + a.AsArticulated()).IsApprox(expected));
  EXPECT_TRUE((a.AsArticulated() + a).IsApprox(expected));
}

TEST(InertiaArithmetic, InertiaMinusInertiaOperatorIsZeroForSameBody) {
  Inertia<float> a = ValidInertia();
  EXPECT_TRUE((a - a.AsArticulated()).IsZero());
  EXPECT_TRUE((a.AsArticulated() - a).IsZero());
}

// Scalar algebra: *, /, *=, /=.
TEST(InertiaScalarAlgebra, MultiplyAndDivide) {
  Inertia<float> a = ValidInertia();

  Inertia<float> scaled = a * 2.0F;
  EXPECT_FLOAT_EQ(scaled.Mass(), 4.0F);
  EXPECT_FLOAT_EQ(scaled.Ixx(), 4.0F);

  Inertia<float> divided = scaled / 2.0F;
  EXPECT_TRUE(divided.IsApprox(a));
}

TEST(InertiaScalarAlgebra, CompoundMultiplyAndDivide) {
  Inertia<float> a = ValidInertia();
  Inertia<float> in_place = a;
  in_place *= 2.0F;
  EXPECT_TRUE(in_place.IsApprox(a * 2.0F));
  in_place /= 2.0F;
  EXPECT_TRUE(in_place.IsApprox(a));
}

// Apply(SpatialVelocity)/Apply(SpatialAcceleration): Inertia's own
// specialized formula must agree with going through the generic
// InertiaOperator::Apply (data_ * v, via AsArticulated()) -- two
// independently-written implementations of the same physics.
TEST(InertiaApply, VelocityMatchesArticulatedForm) {
  Inertia<float> i = ValidInertia();
  SpatialVelocity<float> v(
      Vector3<float>(0.1F, 0.2F, 0.3F), Vector3<float>(1.0F, -2.0F, 0.5F)
  );

  SpatialMomentum<float> via_sparse = i.Apply(v);
  SpatialMomentum<float> via_operator = i.AsArticulated().Apply(v);
  EXPECT_TRUE(via_sparse.IsApprox(via_operator, 1e-4F));
}

TEST(InertiaApply, AccelerationMatchesArticulatedForm) {
  Inertia<float> i = ValidInertia();
  SpatialAcceleration<float> a(
      Vector3<float>(0.1F, 0.2F, 0.3F), Vector3<float>(1.0F, -2.0F, 0.5F)
  );

  SpatialForce<float> via_sparse = i.Apply(a);
  SpatialForce<float> via_operator = i.AsArticulated().Apply(a);
  EXPECT_TRUE(via_sparse.IsApprox(via_operator, 1e-4F));
}

// Inverse(): applying it to whatever Apply(velocity) produced must
// recover the original velocity -- M^-1 * (M * v) == v.
TEST(InertiaInverse, RoundTripsThroughApply) {
  Inertia<float> i = ValidInertia();
  SpatialVelocity<float> v(
      Vector3<float>(0.1F, 0.2F, 0.3F), Vector3<float>(1.0F, -2.0F, 0.5F)
  );

  SpatialMomentum<float> p = i.Apply(v);
  SpatialVelocity<float> recovered = i.Inverse().Apply(p);
  EXPECT_TRUE(recovered.IsApprox(v, 1e-3F));
}

TEST(InertiaInverse, MatrixProductIsIdentity) {
  Inertia<float> i = ValidInertia();
  Matrix6x6<float> product = i.AsMatrix() * i.Inverse().AsMatrix();
  EXPECT_TRUE(product.IsApprox(Matrix6x6<float>::Identity(), 1e-3F));
}

// ===== InertiaOperator<T, Inverted> =====

TEST(InertiaOperatorConstruction, DefaultIsZero) {
  ArticulatedInertiaOperator<float> op;
  EXPECT_TRUE(op.IsZero());
}

TEST(InertiaOperatorConstruction, FromMatrix6x6) {
  Matrix6x6<float> m = ValidInertia().AsMatrix();
  ArticulatedInertiaOperator<float> op(m);
  EXPECT_TRUE(op.AsMatrix().IsApprox(m));
}

TEST(InertiaOperatorStaticConstructors, ZeroAndSetZero) {
  EXPECT_TRUE(ArticulatedInertiaOperator<float>::Zero().IsZero());

  ArticulatedInertiaOperator<float> op = ValidInertia().AsArticulated();
  op.SetZero();
  EXPECT_TRUE(op.IsZero());
}

TEST(InertiaOperatorStaticConstructors, IdentityAndSetIdentity) {
  ArticulatedInertiaOperator<float> id =
      ArticulatedInertiaOperator<float>::Identity();
  EXPECT_TRUE(id.AsMatrix().IsApprox(Matrix6x6<float>::Identity()));

  ArticulatedInertiaOperator<float> op = ValidInertia().AsArticulated();
  op.SetIdentity();
  EXPECT_TRUE(op.IsApprox(id));
}

// Access: ToTuple, AsMatrix, IsInverse.
TEST(InertiaOperatorAccess, ToTupleAsMatrixAndIsInverse) {
  Matrix6x6<float> m = ValidInertia().AsMatrix();
  ArticulatedInertiaOperator<float> op(m);

  auto [tm] = op.ToTuple();
  EXPECT_TRUE(tm.IsApprox(m));
  EXPECT_TRUE(op.AsMatrix().IsApprox(m));
  EXPECT_FALSE(op.IsInverse());

  InvertedArticulatedInertiaOperator<float> inv(m);
  EXPECT_TRUE(inv.IsInverse());
}

// AsSparse() inverts AsArticulated(): the matrix built from a real
// Inertia has exactly the block structure (symmetric top-left, skew
// top-right/bottom-left, mass*Identity bottom-right) AsSparse() requires.
TEST(InertiaOperatorAsSparse, RoundTripsThroughAsArticulated) {
  Inertia<float> i = ValidInertia();
  EXPECT_TRUE(i.AsArticulated().AsSparse().IsApprox(i));

  Inertia<float> shifted = ShiftedInertia();
  EXPECT_TRUE(shifted.AsArticulated().AsSparse().IsApprox(shifted, 1e-3F));
}

TEST(InertiaOperatorComparison, EqualityAndInequality) {
  ArticulatedInertiaOperator<float> a = ValidInertia().AsArticulated();
  ArticulatedInertiaOperator<float> b = a;
  ArticulatedInertiaOperator<float> c =
      ArticulatedInertiaOperator<float>::Identity();

  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a != b);
  EXPECT_TRUE(a != c);
  EXPECT_FALSE(a == c);
}

TEST(InertiaOperatorComparison, IsApproxAndIsZero) {
  ArticulatedInertiaOperator<float> a = ValidInertia().AsArticulated();
  EXPECT_TRUE(a.IsApprox(a * 1.0000001F));
  EXPECT_TRUE(ArticulatedInertiaOperator<float>::Zero().IsZero());
  EXPECT_FALSE(a.IsZero());
}

// Elementwise arithmetic: op+op, op-op, and their in-place forms.
TEST(InertiaOperatorArithmetic, AdditionAndSubtraction) {
  ArticulatedInertiaOperator<float> a = ValidInertia().AsArticulated();

  ArticulatedInertiaOperator<float> sum = a + a;
  EXPECT_TRUE(sum.IsApprox(a * 2.0F));

  ArticulatedInertiaOperator<float> diff = sum - a;
  EXPECT_TRUE(diff.IsApprox(a));

  ArticulatedInertiaOperator<float> in_place = a;
  in_place += a;
  EXPECT_TRUE(in_place.IsApprox(sum));
  in_place -= a;
  EXPECT_TRUE(in_place.IsApprox(a));
}

TEST(InertiaOperatorArithmetic, CompoundAddAndSubtractInertia) {
  Inertia<float> i = ValidInertia();
  ArticulatedInertiaOperator<float> op =
      ArticulatedInertiaOperator<float>::Zero();
  op += i;
  EXPECT_TRUE(op.IsApprox(i.AsArticulated()));
  op -= i;
  EXPECT_TRUE(op.IsZero());
}

// Scalar algebra: *, /, *=, /=.
TEST(InertiaOperatorScalarAlgebra, MultiplyAndDivide) {
  ArticulatedInertiaOperator<float> a = ValidInertia().AsArticulated();

  ArticulatedInertiaOperator<float> scaled = a * 2.0F;
  ArticulatedInertiaOperator<float> divided = scaled / 2.0F;
  EXPECT_TRUE(divided.IsApprox(a));

  ArticulatedInertiaOperator<float> in_place = a;
  in_place *= 2.0F;
  EXPECT_TRUE(in_place.IsApprox(scaled));
  in_place /= 2.0F;
  EXPECT_TRUE(in_place.IsApprox(a));
}

// Apply: non-inverted operators map velocity->momentum and
// acceleration->force; inverted operators map momentum->velocity and
// force->acceleration. Round-tripping through Inverse() must recover the
// original spatial vector.
TEST(InertiaOperatorApply, NonInvertedVelocityAndAcceleration) {
  ArticulatedInertiaOperator<float> op = ValidInertia().AsArticulated();
  SpatialVelocity<float> v(
      Vector3<float>(0.1F, 0.2F, 0.3F), Vector3<float>(1.0F, -2.0F, 0.5F)
  );
  SpatialAcceleration<float> a(
      Vector3<float>(0.1F, 0.2F, 0.3F), Vector3<float>(1.0F, -2.0F, 0.5F)
  );

  SpatialMomentum<float> p = op.Apply(v);
  SpatialForce<float> f = op.Apply(a);
  EXPECT_TRUE(p.AsVector6().IsApprox((op.AsMatrix() * v).AsVector6(), 1e-4F));
  EXPECT_TRUE(f.AsVector6().IsApprox((op.AsMatrix() * a).AsVector6(), 1e-4F));
}

TEST(InertiaOperatorApply, InvertedRoundTripsMomentumAndForce) {
  InvertedArticulatedInertiaOperator<float> inv = ValidInertia().Inverse();
  ArticulatedInertiaOperator<float> op = ValidInertia().AsArticulated();

  SpatialVelocity<float> v(
      Vector3<float>(0.1F, 0.2F, 0.3F), Vector3<float>(1.0F, -2.0F, 0.5F)
  );
  SpatialMomentum<float> p = op.Apply(v);
  SpatialVelocity<float> recovered = inv.Apply(p);
  EXPECT_TRUE(recovered.IsApprox(v, 1e-3F));

  SpatialAcceleration<float> a(
      Vector3<float>(0.1F, 0.2F, 0.3F), Vector3<float>(1.0F, -2.0F, 0.5F)
  );
  SpatialForce<float> f = op.Apply(a);
  SpatialAcceleration<float> recovered_a = inv.Apply(f);
  EXPECT_TRUE(recovered_a.IsApprox(a, 1e-3F));
}

// Matrix operations: op*op, op*Matrix6x6 (member and friend forms),
// Transpose/TransposeInPlace, Inverse.
TEST(InertiaOperatorMatrixOps, MultiplyByOperatorAndMatrix) {
  ArticulatedInertiaOperator<float> op = ValidInertia().AsArticulated();
  Matrix6x6<float> identity = Matrix6x6<float>::Identity();

  EXPECT_TRUE((op * ArticulatedInertiaOperator<float>::Identity()).IsApprox(op)
  );
  EXPECT_TRUE((op * identity).AsMatrix().IsApprox(op.AsMatrix()));
  EXPECT_TRUE((identity * op).AsMatrix().IsApprox(op.AsMatrix()));
}

// A real rigid-body inertia matrix is symmetric, so Transpose() must be a
// no-op for one.
TEST(InertiaOperatorMatrixOps, TransposeOfSymmetricInertiaIsNoOp) {
  ArticulatedInertiaOperator<float> op = ValidInertia().AsArticulated();
  EXPECT_TRUE(op.Transpose().IsApprox(op));

  ArticulatedInertiaOperator<float> in_place = op;
  in_place.TransposeInPlace();
  EXPECT_TRUE(in_place.IsApprox(op));
}

TEST(InertiaOperatorMatrixOps, InverseFlipsInvertedFlagAndProductIsIdentity) {
  ArticulatedInertiaOperator<float> op = ValidInertia().AsArticulated();
  InvertedArticulatedInertiaOperator<float> inv = op.Inverse();
  EXPECT_TRUE(inv.IsInverse());

  Matrix6x6<float> product = op.AsMatrix() * inv.AsMatrix();
  EXPECT_TRUE(product.IsApprox(Matrix6x6<float>::Identity(), 1e-3F));
}

// MaskedInverse, exercised through MaskArchetype (see
// tests/support/mask_archetype.hpp) to prove it only depends on the
// MaskLike interface: an all-active mask must behave exactly like an
// ordinary Inverse().
TEST(InertiaOperatorMaskedInverse, AllActiveMatchesOrdinaryInverse) {
  ArticulatedInertiaOperator<float> op = ValidInertia().AsArticulated();
  InvertedArticulatedInertiaOperator<float> expected = op.Inverse();

  MaskArchetype<6> all_active = MaskArchetype<6>::Ones();
  InvertedArticulatedInertiaOperator<float> masked =
      op.MaskedInverse(all_active);
  EXPECT_TRUE(masked.IsApprox(expected, 1e-3F));
}
