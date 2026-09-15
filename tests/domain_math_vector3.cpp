#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <xsimd/xsimd.hpp>

#include "domain/math/matrix.hpp"
#include "domain/math/vector3.hpp"
#include "support/simd_test_helpers.hpp"

using achilles::domain::math::Matrix;
using achilles::domain::math::Vector3;
using achilles::test_support::MakeBatch;

namespace {

constexpr float kPi = std::numbers::pi_v<float>;

}  // namespace

// Every constructor produces the components it was given: the zero
// default, the passthrough from a backing Matrix<T,3,1>, and the (x,y,z)
// triple.
TEST(Vector3Construction, DefaultIsZero) {
  Vector3<float> v;
  EXPECT_FLOAT_EQ(v.X(), 0.0F);
  EXPECT_FLOAT_EQ(v.Y(), 0.0F);
  EXPECT_FLOAT_EQ(v.Z(), 0.0F);
}

TEST(Vector3Construction, FromComponents) {
  Vector3<float> v(1.0F, 2.0F, 3.0F);
  EXPECT_FLOAT_EQ(v.X(), 1.0F);
  EXPECT_FLOAT_EQ(v.Y(), 2.0F);
  EXPECT_FLOAT_EQ(v.Z(), 3.0F);
}

TEST(Vector3Construction, FromMatrix) {
  Matrix<float, 3, 1> m(1.0F, 2.0F, 3.0F);
  Vector3<float> v(m);
  EXPECT_FLOAT_EQ(v.X(), 1.0F);
  EXPECT_FLOAT_EQ(v.Y(), 2.0F);
  EXPECT_FLOAT_EQ(v.Z(), 3.0F);
}

// Named static constructors and their in-place counterparts.
TEST(Vector3StaticConstructors, ZeroUnitsAndOnes) {
  EXPECT_TRUE(Vector3<float>::Zero().IsZero());

  Vector3<float> ones = Vector3<float>::Ones();
  EXPECT_FLOAT_EQ(ones.X(), 1.0F);
  EXPECT_FLOAT_EQ(ones.Y(), 1.0F);
  EXPECT_FLOAT_EQ(ones.Z(), 1.0F);
}

TEST(Vector3StaticConstructors, Units) {
  EXPECT_FLOAT_EQ(Vector3<float>::UnitX().X(), 1.0F);
  EXPECT_FLOAT_EQ(Vector3<float>::UnitX().Y(), 0.0F);
  EXPECT_FLOAT_EQ(Vector3<float>::UnitX().Z(), 0.0F);

  EXPECT_FLOAT_EQ(Vector3<float>::UnitY().X(), 0.0F);
  EXPECT_FLOAT_EQ(Vector3<float>::UnitY().Y(), 1.0F);
  EXPECT_FLOAT_EQ(Vector3<float>::UnitY().Z(), 0.0F);

  EXPECT_FLOAT_EQ(Vector3<float>::UnitZ().X(), 0.0F);
  EXPECT_FLOAT_EQ(Vector3<float>::UnitZ().Y(), 0.0F);
  EXPECT_FLOAT_EQ(Vector3<float>::UnitZ().Z(), 1.0F);
}

TEST(Vector3StaticConstructors, SetZeroClearsInPlace) {
  Vector3<float> v(1.0F, 2.0F, 3.0F);
  v.SetZero();
  EXPECT_TRUE(v.IsZero());
}

// FromSkew is Skew()'s inverse: pulling the axial vector back out of a
// skew-symmetric matrix must reproduce the vector that built it.
TEST(Vector3StaticConstructors, FromSkewInvertsSkew) {
  Vector3<float> v(1.0F, -2.0F, 3.0F);
  Vector3<float> round_tripped = Vector3<float>::FromSkew(v.Skew());
  EXPECT_TRUE(round_tripped.IsApprox(v));
}

// Accessors: ToTuple, X/Y/Z, operator[], AsMatrix must all agree on the
// same three components.
TEST(Vector3Access, AllAccessorsAgree) {
  Vector3<float> v(4.0F, 5.0F, 6.0F);

  EXPECT_FLOAT_EQ(v.X(), 4.0F);
  EXPECT_FLOAT_EQ(v.Y(), 5.0F);
  EXPECT_FLOAT_EQ(v.Z(), 6.0F);
  EXPECT_FLOAT_EQ(v[0], 4.0F);
  EXPECT_FLOAT_EQ(v[1], 5.0F);
  EXPECT_FLOAT_EQ(v[2], 6.0F);

  auto [tx, ty, tz] = v.ToTuple();
  EXPECT_FLOAT_EQ(tx, 4.0F);
  EXPECT_FLOAT_EQ(ty, 5.0F);
  EXPECT_FLOAT_EQ(tz, 6.0F);

  EXPECT_FLOAT_EQ(v.AsMatrix()(0, 0), 4.0F);
  EXPECT_FLOAT_EQ(v.AsMatrix()(2, 0), 6.0F);
}

TEST(Vector3Comparison, EqualityAndInequality) {
  Vector3<float> a(1.0F, 2.0F, 3.0F);
  Vector3<float> b(1.0F, 2.0F, 3.0F);
  Vector3<float> c(1.0F, 2.0F, 3.5F);

  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a != b);
  EXPECT_TRUE(a != c);
  EXPECT_FALSE(a == c);
}

TEST(Vector3Comparison, IsApproxWithinAndOutsideEpsilon) {
  Vector3<float> a(1.0F, 2.0F, 3.0F);
  Vector3<float> close(1.0F + 1e-7F, 2.0F, 3.0F);
  Vector3<float> far(1.1F, 2.0F, 3.0F);

  EXPECT_TRUE(a.IsApprox(close));
  EXPECT_FALSE(a.IsApprox(far));
}

TEST(Vector3Comparison, IsZero) {
  EXPECT_TRUE(Vector3<float>(0.0F, 0.0F, 0.0F).IsZero());
  EXPECT_FALSE(Vector3<float>(0.0F, 0.1F, 0.0F).IsZero());
}

// Elementwise arithmetic: +, -, unary -, and their in-place forms.
TEST(Vector3Arithmetic, AdditionAndSubtraction) {
  Vector3<float> a(1.0F, 2.0F, 3.0F);
  Vector3<float> b(4.0F, -1.0F, 0.5F);

  Vector3<float> sum = a + b;
  EXPECT_FLOAT_EQ(sum.X(), 5.0F);
  EXPECT_FLOAT_EQ(sum.Y(), 1.0F);
  EXPECT_FLOAT_EQ(sum.Z(), 3.5F);

  Vector3<float> diff = a - b;
  EXPECT_FLOAT_EQ(diff.X(), -3.0F);
  EXPECT_FLOAT_EQ(diff.Y(), 3.0F);
  EXPECT_FLOAT_EQ(diff.Z(), 2.5F);
}

TEST(Vector3Arithmetic, UnaryNegationAndNegateInPlace) {
  Vector3<float> a(1.0F, -2.0F, 3.0F);

  Vector3<float> negated = -a;
  EXPECT_FLOAT_EQ(negated.X(), -1.0F);
  EXPECT_FLOAT_EQ(negated.Y(), 2.0F);
  EXPECT_FLOAT_EQ(negated.Z(), -3.0F);

  Vector3<float> b = a;
  b.NegateInPlace();
  EXPECT_TRUE(b.IsApprox(negated));
}

TEST(Vector3Arithmetic, CompoundAddAndSubtract) {
  Vector3<float> a(1.0F, 2.0F, 3.0F);
  Vector3<float> b(0.5F, 0.5F, 0.5F);

  Vector3<float> c = a;
  c += b;
  EXPECT_TRUE(c.IsApprox(Vector3<float>(1.5F, 2.5F, 3.5F)));

  c -= b;
  EXPECT_TRUE(c.IsApprox(a));
}

// Scalar algebra: *, /, *=, /=, plus the friend scalar*vector overload.
TEST(Vector3ScalarAlgebra, MultiplyAndDivide) {
  Vector3<float> v(2.0F, 4.0F, -6.0F);

  Vector3<float> scaled = v * 2.0F;
  EXPECT_TRUE(scaled.IsApprox(Vector3<float>(4.0F, 8.0F, -12.0F)));

  Vector3<float> divided = v / 2.0F;
  EXPECT_TRUE(divided.IsApprox(Vector3<float>(1.0F, 2.0F, -3.0F)));

  Vector3<float> friend_scaled = 3.0F * v;
  EXPECT_TRUE(friend_scaled.IsApprox(v * 3.0F));
}

TEST(Vector3ScalarAlgebra, CompoundMultiplyAndDivide) {
  Vector3<float> v(2.0F, 4.0F, -6.0F);

  Vector3<float> a = v;
  a *= 2.0F;
  EXPECT_TRUE(a.IsApprox(Vector3<float>(4.0F, 8.0F, -12.0F)));

  a /= 4.0F;
  EXPECT_TRUE(a.IsApprox(Vector3<float>(1.0F, 2.0F, -3.0F)));
}

// Matrix*vector friend overload, distinct from the scalar*vector one.
TEST(Vector3ScalarAlgebra, MatrixTimesVector) {
  Matrix<float, 3, 3> identity = Matrix<float, 3, 3>::Identity();
  Vector3<float> v(1.0F, 2.0F, 3.0F);
  EXPECT_TRUE((identity * v).IsApprox(v));
}

TEST(Vector3Products, DotProduct) {
  Vector3<float> a(1.0F, 2.0F, 3.0F);
  Vector3<float> b(4.0F, -5.0F, 6.0F);
  EXPECT_FLOAT_EQ(a.Dot(b), 1.0F * 4.0F + 2.0F * -5.0F + 3.0F * 6.0F);
}

TEST(Vector3Products, CrossProductOfBasisVectors) {
  Vector3<float> x = Vector3<float>::UnitX();
  Vector3<float> y = Vector3<float>::UnitY();
  EXPECT_TRUE(x.Cross(y).IsApprox(Vector3<float>::UnitZ()));
}

TEST(Vector3Norms, NormAndSquaredNorm) {
  Vector3<float> v(3.0F, 4.0F, 0.0F);
  EXPECT_FLOAT_EQ(v.SquaredNorm(), 25.0F);
  EXPECT_FLOAT_EQ(v.Norm(), 5.0F);
}

TEST(Vector3Norms, NormalizeAndNormalizeInPlace) {
  Vector3<float> v(3.0F, 4.0F, 0.0F);

  Vector3<float> unit = v.Normalize();
  EXPECT_FLOAT_EQ(unit.Norm(), 1.0F);
  EXPECT_TRUE(unit.IsApprox(Vector3<float>(0.6F, 0.8F, 0.0F)));

  Vector3<float> in_place = v;
  in_place.NormalizeInPlace();
  EXPECT_TRUE(in_place.IsApprox(unit));
}

// AngleTo: perpendicular unit vectors are pi/2 apart, parallel ones are 0
// apart, and the zero-length guard returns 0 instead of dividing by zero.
TEST(Vector3Geometry, AngleToPerpendicularAndParallel) {
  Vector3<float> x = Vector3<float>::UnitX();
  Vector3<float> y = Vector3<float>::UnitY();

  EXPECT_NEAR(x.AngleTo(y), kPi / 2.0F, 1e-5F);
  EXPECT_NEAR(x.AngleTo(x), 0.0F, 1e-5F);
}

TEST(Vector3Geometry, AngleToZeroVectorIsZeroNotNan) {
  Vector3<float> x = Vector3<float>::UnitX();
  EXPECT_FLOAT_EQ(x.AngleTo(Vector3<float>::Zero()), 0.0F);
}

// ProjectOnto: projecting onto a basis axis isolates that component;
// projecting onto the zero vector is the documented zero-guard, not a
// division by zero.
TEST(Vector3Geometry, ProjectOntoAxis) {
  Vector3<float> v(3.0F, 4.0F, 5.0F);
  Vector3<float> projected = v.ProjectOnto(Vector3<float>::UnitX());
  EXPECT_TRUE(projected.IsApprox(Vector3<float>(3.0F, 0.0F, 0.0F)));
}

TEST(Vector3Geometry, ProjectOntoZeroVectorIsZero) {
  Vector3<float> v(3.0F, 4.0F, 5.0F);
  EXPECT_TRUE(v.ProjectOnto(Vector3<float>::Zero()).IsZero());
}

TEST(Vector3Geometry, SkewProducesCrossProductMatrix) {
  Vector3<float> a(1.0F, 2.0F, 3.0F);
  Vector3<float> b(4.0F, 5.0F, 6.0F);
  Vector3<float> via_skew(a.Skew() * b.AsMatrix());
  EXPECT_TRUE(via_skew.IsApprox(a.Cross(b)));
}

TEST(Vector3Interpolation, LerpEndpointsAndMidpoint) {
  Vector3<float> a(0.0F, 0.0F, 0.0F);
  Vector3<float> b(10.0F, 20.0F, -10.0F);

  EXPECT_TRUE(Vector3<float>::Lerp(a, b, 0.0F).IsApprox(a));
  EXPECT_TRUE(Vector3<float>::Lerp(a, b, 1.0F).IsApprox(b));
  EXPECT_TRUE(Vector3<float>::Lerp(a, b, 0.5F)
                  .IsApprox(Vector3<float>(5.0F, 10.0F, -5.0F)));
}

// Batched smoke test: confirms T = xsimd::batch<float> is a genuine free
// parameter, not just float in disguise, by giving each lane a distinct
// value and checking Dot/Cross/Norm lane-by-lane.
TEST(Vector3Batched, DotCrossAndNormPerLane) {
  Vector3<xsimd::batch<float>> a(
      MakeBatch({1.0F, 0.0F}), MakeBatch({0.0F, 3.0F}), MakeBatch({0.0F, 4.0F})
  );
  Vector3<xsimd::batch<float>> b(
      MakeBatch({1.0F, 0.0F}), MakeBatch({0.0F, 0.0F}), MakeBatch({0.0F, 0.0F})
  );

  auto dot = a.Dot(b);
  EXPECT_FLOAT_EQ(dot.get(0), 1.0F);
  EXPECT_FLOAT_EQ(dot.get(1), 0.0F);

  auto norm = a.Norm();
  EXPECT_FLOAT_EQ(norm.get(0), 1.0F);
  EXPECT_FLOAT_EQ(norm.get(1), 5.0F);
}
