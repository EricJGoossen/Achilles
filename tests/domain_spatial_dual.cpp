#include <gtest/gtest.h>

#include <cstddef>

#include "domain/math/matrix.hpp"
#include "domain/math/vector3.hpp"
#include "domain/math/vector6.hpp"
#include "domain/spatial/dual.hpp"

using achilles::domain::math::Matrix;
using achilles::domain::math::Vector3;
using achilles::domain::math::Vector6;
using achilles::domain::spatial::SpatialAcceleration;
using achilles::domain::spatial::SpatialForce;
using achilles::domain::spatial::SpatialMomentum;
using achilles::domain::spatial::SpatialPosition;
using achilles::domain::spatial::SpatialVelocity;

// Dual<DerivedT, T> (domain/spatial/dual.hpp) is a CRTP base shared by
// SpatialPosition/Velocity/Acceleration/Momentum/Force -- every method it
// defines is identical machinery across all five, so the shared surface
// below is tested once through SpatialPosition (the simplest derived type:
// it adds nothing of its own). Each derived type still gets its own smoke
// test further down, to catch anything specific to that CRTP
// instantiation, and the methods SpatialVelocity/SpatialForce add on top
// (CrossForce, Cross, the cross-type Dot) get their own dedicated tests.

TEST(DualConstruction, DefaultIsZero) {
  SpatialPosition<float> p;
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_FLOAT_EQ(p[i], 0.0F);
  }
}

TEST(DualConstruction, FromSixComponents) {
  SpatialPosition<float> p(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  EXPECT_FLOAT_EQ(p.Roll(), 1.0F);
  EXPECT_FLOAT_EQ(p.Pitch(), 2.0F);
  EXPECT_FLOAT_EQ(p.Yaw(), 3.0F);
  EXPECT_FLOAT_EQ(p.X(), 4.0F);
  EXPECT_FLOAT_EQ(p.Y(), 5.0F);
  EXPECT_FLOAT_EQ(p.Z(), 6.0F);
}

TEST(DualConstruction, FromAngularAndLinearVectors) {
  Vector3<float> angular(1.0F, 2.0F, 3.0F);
  Vector3<float> linear(4.0F, 5.0F, 6.0F);
  SpatialPosition<float> p(angular, linear);
  EXPECT_TRUE(p.Angular().IsApprox(angular));
  EXPECT_TRUE(p.Linear().IsApprox(linear));
}

TEST(DualConstruction, FromVector6) {
  Vector6<float> v(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  SpatialPosition<float> p(v);
  EXPECT_TRUE(p.AsVector6().IsApprox(v));
}

TEST(DualStaticConstructors, ZeroUnitsAndOnes) {
  EXPECT_TRUE(SpatialPosition<float>::Zero().IsZero());

  EXPECT_FLOAT_EQ(SpatialPosition<float>::UnitRoll().Roll(), 1.0F);
  EXPECT_FLOAT_EQ(SpatialPosition<float>::UnitPitch().Pitch(), 1.0F);
  EXPECT_FLOAT_EQ(SpatialPosition<float>::UnitYaw().Yaw(), 1.0F);
  EXPECT_FLOAT_EQ(SpatialPosition<float>::UnitX().X(), 1.0F);
  EXPECT_FLOAT_EQ(SpatialPosition<float>::UnitY().Y(), 1.0F);
  EXPECT_FLOAT_EQ(SpatialPosition<float>::UnitZ().Z(), 1.0F);

  SpatialPosition<float> ones = SpatialPosition<float>::Ones();
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_FLOAT_EQ(ones[i], 1.0F);
  }
}

TEST(DualStaticConstructors, SetZeroClearsInPlace) {
  SpatialPosition<float> p(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  p.SetZero();
  EXPECT_TRUE(p.IsZero());
}

// Access: ToTuple, Roll/Pitch/Yaw/X/Y/Z, Linear/Angular, operator[],
// AsVector6, AsMatrix must all agree on the same six components.
TEST(DualAccess, AllAccessorsAgree) {
  SpatialPosition<float> p(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);

  auto [v6] = p.ToTuple();
  EXPECT_TRUE(v6.IsApprox(p.AsVector6()));

  EXPECT_TRUE(p.Angular().IsApprox(Vector3<float>(1.0F, 2.0F, 3.0F)));
  EXPECT_TRUE(p.Linear().IsApprox(Vector3<float>(4.0F, 5.0F, 6.0F)));

  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_FLOAT_EQ(p[i], p.AsMatrix()(i, 0));
  }
}

// As<OtherDerivedT>() re-tags the same six components as a different
// spatial type without touching the values.
TEST(DualConversion, AsReinterpretsAsAnotherDerivedType) {
  SpatialPosition<float> p(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  SpatialVelocity<float> as_velocity = p.As<SpatialVelocity>();
  EXPECT_TRUE(as_velocity.AsVector6().IsApprox(p.AsVector6()));
}

TEST(DualComparison, EqualityAndInequality) {
  SpatialPosition<float> a(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  SpatialPosition<float> b = a;
  SpatialPosition<float> c(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.5F);

  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a != b);
  EXPECT_TRUE(a != c);
  EXPECT_FALSE(a == c);
}

TEST(DualComparison, IsApproxWithinAndOutsideEpsilon) {
  SpatialPosition<float> a(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  SpatialPosition<float> close = a;
  close += SpatialPosition<float>(1e-7F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
  SpatialPosition<float> far = a;
  far += SpatialPosition<float>(0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);

  EXPECT_TRUE(a.IsApprox(close));
  EXPECT_FALSE(a.IsApprox(far));
}

TEST(DualComparison, IsZero) {
  EXPECT_TRUE(SpatialPosition<float>::Zero().IsZero());
  EXPECT_FALSE(SpatialPosition<float>::UnitX().IsZero());
}

// Elementwise arithmetic: +, -, unary -, NegateInPlace, +=, -=.
TEST(DualArithmetic, AdditionAndSubtraction) {
  SpatialPosition<float> a(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  SpatialPosition<float> b(6.0F, 5.0F, 4.0F, 3.0F, 2.0F, 1.0F);

  SpatialPosition<float> sum = a + b;
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_FLOAT_EQ(sum[i], 7.0F);
  }

  SpatialPosition<float> diff = a - b;
  EXPECT_FLOAT_EQ(diff[0], -5.0F);
  EXPECT_FLOAT_EQ(diff[5], 5.0F);
}

TEST(DualArithmetic, UnaryNegationAndNegateInPlace) {
  SpatialPosition<float> a(1.0F, -2.0F, 3.0F, -4.0F, 5.0F, -6.0F);

  SpatialPosition<float> negated = -a;
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_FLOAT_EQ(negated[i], -a[i]);
  }

  SpatialPosition<float> b = a;
  b.NegateInPlace();
  EXPECT_TRUE(b.IsApprox(negated));
}

TEST(DualArithmetic, CompoundAddAndSubtract) {
  SpatialPosition<float> a(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  SpatialPosition<float> b = SpatialPosition<float>::Ones();

  SpatialPosition<float> c = a;
  c += b;
  EXPECT_TRUE(c.IsApprox(a + b));
  c -= b;
  EXPECT_TRUE(c.IsApprox(a));
}

// Scalar algebra: *, /, *=, /=, and the friend scalar*Derived overload.
TEST(DualScalarAlgebra, MultiplyAndDivide) {
  SpatialPosition<float> p(2.0F, 4.0F, 6.0F, 8.0F, 10.0F, 12.0F);

  SpatialPosition<float> scaled = p * 0.5F;
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_FLOAT_EQ(scaled[i], p[i] * 0.5F);
  }

  SpatialPosition<float> divided = p / 2.0F;
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_FLOAT_EQ(divided[i], p[i] / 2.0F);
  }

  EXPECT_TRUE((3.0F * p).IsApprox(p * 3.0F));
}

TEST(DualScalarAlgebra, CompoundMultiplyAndDivide) {
  SpatialPosition<float> p = SpatialPosition<float>::Ones();
  SpatialPosition<float> a = p;
  a *= 4.0F;
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_FLOAT_EQ(a[i], 4.0F);
  }
  a /= 2.0F;
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_FLOAT_EQ(a[i], 2.0F);
  }
}

// Friend Matrix6x6*Derived overload, distinct from the scalar*Derived one.
TEST(DualScalarAlgebra, MatrixTimesDerived) {
  Matrix<float, 6, 6> identity = Matrix<float, 6, 6>::Identity();
  SpatialPosition<float> p(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  EXPECT_TRUE((identity * p).IsApprox(p));
}

TEST(DualProducts, DotProduct) {
  SpatialPosition<float> a = SpatialPosition<float>::UnitX();
  SpatialPosition<float> b(100.0F, 100.0F, 100.0F, 3.0F, 100.0F, 100.0F);
  EXPECT_FLOAT_EQ(a.Dot(b), 3.0F);
}

TEST(DualNorms, NormAndSquaredNorm) {
  SpatialPosition<float> p(0.0F, 0.0F, 0.0F, 3.0F, 4.0F, 0.0F);
  EXPECT_FLOAT_EQ(p.SquaredNorm(), 25.0F);
  EXPECT_FLOAT_EQ(p.Norm(), 5.0F);
}

TEST(DualNorms, NormalizeAndNormalizeInPlace) {
  SpatialPosition<float> p(0.0F, 0.0F, 0.0F, 3.0F, 4.0F, 0.0F);

  SpatialPosition<float> unit = p.Normalize();
  EXPECT_FLOAT_EQ(unit.Norm(), 1.0F);

  SpatialPosition<float> in_place = p;
  in_place.NormalizeInPlace();
  EXPECT_TRUE(in_place.IsApprox(unit));
}

// ProjectOnto: projecting onto a basis axis isolates that component;
// projecting onto the zero Dual is the documented zero-guard.
TEST(DualGeometry, ProjectOntoAxis) {
  // UnitX() names Vector6's D component (see Dual::UnitX() -> UnitD()),
  // which is p's 4th constructor argument -- p.X() == 4.0F -- so the
  // projection scales UnitX() by 4, not 1.
  SpatialPosition<float> p(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  SpatialPosition<float> projected =
      p.ProjectOnto(SpatialPosition<float>::UnitX());
  EXPECT_TRUE(projected.IsApprox(SpatialPosition<float>::UnitX() * 4.0F));
}

TEST(DualGeometry, ProjectOntoZeroIsZero) {
  SpatialPosition<float> p(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  EXPECT_TRUE(p.ProjectOnto(SpatialPosition<float>::Zero()).IsZero());
}

TEST(DualInterpolation, LerpEndpointsAndMidpoint) {
  SpatialPosition<float> a = SpatialPosition<float>::Zero();
  SpatialPosition<float> b(2.0F, 4.0F, 6.0F, 8.0F, 10.0F, 12.0F);

  EXPECT_TRUE(SpatialPosition<float>::Lerp(a, b, 0.0F).IsApprox(a));
  EXPECT_TRUE(SpatialPosition<float>::Lerp(a, b, 1.0F).IsApprox(b));
  EXPECT_TRUE(
      SpatialPosition<float>::Lerp(a, b, 0.5F)
          .IsApprox(SpatialPosition<float>(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F))
  );
}

// Smoke tests for the derived types that add nothing beyond Dual<>: proves
// each CRTP instantiation actually builds and does basic arithmetic, since
// the shared-surface tests above only instantiate Dual through
// SpatialPosition.
TEST(DualDerivedTypesSmoke, SpatialAccelerationConstructsAndAdds) {
  SpatialAcceleration<float> a(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  SpatialAcceleration<float> b = SpatialAcceleration<float>::Ones();
  EXPECT_TRUE((a + b).IsApprox(
      SpatialAcceleration<float>(2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F)
  ));
}

TEST(DualDerivedTypesSmoke, SpatialMomentumConstructsAndAdds) {
  SpatialMomentum<float> a(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  SpatialMomentum<float> b = SpatialMomentum<float>::Ones();
  EXPECT_TRUE((a + b).IsApprox(
      SpatialMomentum<float>(2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F)
  ));
}

// SpatialVelocity-specific: CrossForce and Cross.
//
// CrossForce is the dual (force-side) of the velocity cross product: for a
// pure angular velocity omega crossed with a pure linear momentum p (no
// angular part), the resulting force's linear part is omega x p and its
// angular part is zero (there's no linear velocity or angular momentum
// component to contribute).
TEST(SpatialVelocityCrossForce, PureAngularVelocityCrossLinearMomentum) {
  SpatialVelocity<float> omega(Vector3<float>::UnitZ(), Vector3<float>::Zero());
  SpatialMomentum<float> p(Vector3<float>::Zero(), Vector3<float>::UnitX());

  SpatialForce<float> f = omega.CrossForce(p);
  EXPECT_TRUE(f.Angular().IsZero());
  EXPECT_TRUE(f.Linear().IsApprox(
      Vector3<float>::UnitZ().Cross(Vector3<float>::UnitX())
  ));
}

// Cross (velocity x velocity -> acceleration): two pure angular velocities
// produce a pure angular result with the usual cross-product rule; there's
// no linear contribution when both linear parts are zero.
TEST(SpatialVelocityCross, PureAngularVelocities) {
  SpatialVelocity<float> a(Vector3<float>::UnitX(), Vector3<float>::Zero());
  SpatialVelocity<float> b(Vector3<float>::UnitY(), Vector3<float>::Zero());

  SpatialAcceleration<float> result = a.Cross(b);
  EXPECT_TRUE(result.Angular().IsApprox(
      Vector3<float>::UnitX().Cross(Vector3<float>::UnitY())
  ));
  EXPECT_TRUE(result.Linear().IsZero());
}

// SpatialForce::Dot(SpatialAcceleration) is power (force . acceleration
// across both angular and linear parts), distinct from Dual::Dot(Derived)
// which only ever dots two values of the same spatial type.
TEST(SpatialForceDot, DotsAgainstAcceleration) {
  SpatialForce<float> f(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  SpatialAcceleration<float> a(6.0F, 5.0F, 4.0F, 3.0F, 2.0F, 1.0F);

  float expected = 1.0F * 6.0F + 2.0F * 5.0F + 3.0F * 4.0F + 4.0F * 3.0F +
                   5.0F * 2.0F + 6.0F * 1.0F;
  EXPECT_FLOAT_EQ(f.Dot(a), expected);
}
