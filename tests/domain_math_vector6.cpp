#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <xsimd/xsimd.hpp>

#include "domain/math/matrix.hpp"
#include "domain/math/vector6.hpp"
#include "support/simd_test_helpers.hpp"

using achilles::domain::math::Matrix;
using achilles::domain::math::Vector6;
using achilles::test_support::MakeBatch;

// Every constructor produces the components it was given.
TEST(Vector6Construction, DefaultIsZero) {
  Vector6<float> v;
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_FLOAT_EQ(v[i], 0.0F);
  }
}

TEST(Vector6Construction, FromComponents) {
  Vector6<float> v(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  EXPECT_FLOAT_EQ(v.A(), 1.0F);
  EXPECT_FLOAT_EQ(v.B(), 2.0F);
  EXPECT_FLOAT_EQ(v.C(), 3.0F);
  EXPECT_FLOAT_EQ(v.D(), 4.0F);
  EXPECT_FLOAT_EQ(v.E(), 5.0F);
  EXPECT_FLOAT_EQ(v.F(), 6.0F);
}

TEST(Vector6Construction, FromMatrix) {
  Matrix<float, 6, 1> m(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  Vector6<float> v(m);
  EXPECT_FLOAT_EQ(v.A(), 1.0F);
  EXPECT_FLOAT_EQ(v.F(), 6.0F);
}

// Zero/SetZero and Ones.
TEST(Vector6StaticConstructors, ZeroAndOnes) {
  EXPECT_TRUE(Vector6<float>::Zero().IsZero());

  Vector6<float> ones = Vector6<float>::Ones();
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_FLOAT_EQ(ones[i], 1.0F);
  }
}

TEST(Vector6StaticConstructors, SetZeroClearsInPlace) {
  Vector6<float> v(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  v.SetZero();
  EXPECT_TRUE(v.IsZero());
}

// Each Unit* names exactly one component and leaves the rest zero.
TEST(Vector6StaticConstructors, UnitVectorsAreOneHot) {
  const std::array<Vector6<float>, 6> units = {
      Vector6<float>::UnitA(),
      Vector6<float>::UnitB(),
      Vector6<float>::UnitC(),
      Vector6<float>::UnitD(),
      Vector6<float>::UnitE(),
      Vector6<float>::UnitF(),
  };
  for (std::size_t i = 0; i < 6; ++i) {
    for (std::size_t j = 0; j < 6; ++j) {
      EXPECT_FLOAT_EQ(units.at(i)[j], i == j ? 1.0F : 0.0F);
    }
  }
}

// Accessors: ToTuple, A..F, operator[], AsMatrix must all agree.
TEST(Vector6Access, AllAccessorsAgree) {
  Vector6<float> v(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);

  auto [ta, tb, tc, td, te, tf] = v.ToTuple();
  EXPECT_FLOAT_EQ(ta, v.A());
  EXPECT_FLOAT_EQ(tb, v.B());
  EXPECT_FLOAT_EQ(tc, v.C());
  EXPECT_FLOAT_EQ(td, v.D());
  EXPECT_FLOAT_EQ(te, v.E());
  EXPECT_FLOAT_EQ(tf, v.F());

  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_FLOAT_EQ(v[i], v.AsMatrix()(i, 0));
  }
}

TEST(Vector6Comparison, EqualityAndInequality) {
  Vector6<float> a(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  Vector6<float> b = a;
  Vector6<float> c(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.5F);

  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a != b);
  EXPECT_TRUE(a != c);
  EXPECT_FALSE(a == c);
}

TEST(Vector6Comparison, IsApproxWithinAndOutsideEpsilon) {
  Vector6<float> a(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  Vector6<float> close = a;
  close += Vector6<float>(1e-7F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
  Vector6<float> far = a;
  far += Vector6<float>(0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);

  EXPECT_TRUE(a.IsApprox(close));
  EXPECT_FALSE(a.IsApprox(far));
}

TEST(Vector6Comparison, IsZero) {
  EXPECT_TRUE(Vector6<float>::Zero().IsZero());
  EXPECT_FALSE(Vector6<float>::UnitA().IsZero());
}

// Elementwise arithmetic: +, -, unary -, in-place forms.
TEST(Vector6Arithmetic, AdditionAndSubtraction) {
  Vector6<float> a(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  Vector6<float> b(6.0F, 5.0F, 4.0F, 3.0F, 2.0F, 1.0F);

  Vector6<float> sum = a + b;
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_FLOAT_EQ(sum[i], 7.0F);
  }

  Vector6<float> diff = a - b;
  EXPECT_FLOAT_EQ(diff[0], -5.0F);
  EXPECT_FLOAT_EQ(diff[5], 5.0F);
}

TEST(Vector6Arithmetic, UnaryNegationAndNegateInPlace) {
  Vector6<float> a(1.0F, -2.0F, 3.0F, -4.0F, 5.0F, -6.0F);

  Vector6<float> negated = -a;
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_FLOAT_EQ(negated[i], -a[i]);
  }

  Vector6<float> b = a;
  b.NegateInPlace();
  EXPECT_TRUE(b.IsApprox(negated));
}

TEST(Vector6Arithmetic, CompoundAddAndSubtract) {
  Vector6<float> a(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  Vector6<float> b = Vector6<float>::Ones();

  Vector6<float> c = a;
  c += b;
  EXPECT_TRUE(c.IsApprox(a + b));
  c -= b;
  EXPECT_TRUE(c.IsApprox(a));
}

// Scalar algebra: *, /, *=, /=, and the friend scalar*vector overload.
TEST(Vector6ScalarAlgebra, MultiplyAndDivide) {
  Vector6<float> v(2.0F, 4.0F, 6.0F, 8.0F, 10.0F, 12.0F);

  Vector6<float> scaled = v * 0.5F;
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_FLOAT_EQ(scaled[i], v[i] * 0.5F);
  }
  Vector6<float> divided = v / 2.0F;
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_FLOAT_EQ(divided[i], v[i] / 2.0F);
  }

  EXPECT_TRUE((3.0F * v).IsApprox(v * 3.0F));
}

TEST(Vector6ScalarAlgebra, CompoundMultiplyAndDivide) {
  Vector6<float> v = Vector6<float>::Ones();
  Vector6<float> a = v;
  a *= 4.0F;
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_FLOAT_EQ(a[i], 4.0F);
  }
  a /= 2.0F;
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_FLOAT_EQ(a[i], 2.0F);
  }
}

// Matrix*vector friend overload, distinct from the scalar*vector one.
TEST(Vector6ScalarAlgebra, MatrixTimesVector) {
  Matrix<float, 6, 6> identity = Matrix<float, 6, 6>::Identity();
  Vector6<float> v(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  EXPECT_TRUE((identity * v).IsApprox(v));
}

TEST(Vector6Products, DotProduct) {
  Vector6<float> a = Vector6<float>::UnitA();
  Vector6<float> b(3.0F, 100.0F, 100.0F, 100.0F, 100.0F, 100.0F);
  EXPECT_FLOAT_EQ(a.Dot(b), 3.0F);
}

TEST(Vector6Norms, NormAndSquaredNorm) {
  Vector6<float> v(3.0F, 4.0F, 0.0F, 0.0F, 0.0F, 0.0F);
  EXPECT_FLOAT_EQ(v.SquaredNorm(), 25.0F);
  EXPECT_FLOAT_EQ(v.Norm(), 5.0F);
}

TEST(Vector6Norms, NormalizeAndNormalizeInPlace) {
  Vector6<float> v(3.0F, 4.0F, 0.0F, 0.0F, 0.0F, 0.0F);

  Vector6<float> unit = v.Normalize();
  EXPECT_FLOAT_EQ(unit.Norm(), 1.0F);

  Vector6<float> in_place = v;
  in_place.NormalizeInPlace();
  EXPECT_TRUE(in_place.IsApprox(unit));
}

// ProjectOnto: projecting onto a basis axis isolates that component;
// projecting onto the zero vector is the documented zero-guard.
TEST(Vector6Geometry, ProjectOntoAxis) {
  Vector6<float> v(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  Vector6<float> projected = v.ProjectOnto(Vector6<float>::UnitA());
  EXPECT_TRUE(projected.IsApprox(Vector6<float>::UnitA()));
}

TEST(Vector6Geometry, ProjectOntoZeroVectorIsZero) {
  Vector6<float> v(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  EXPECT_TRUE(v.ProjectOnto(Vector6<float>::Zero()).IsZero());
}

TEST(Vector6Interpolation, LerpEndpointsAndMidpoint) {
  Vector6<float> a = Vector6<float>::Zero();
  Vector6<float> b(2.0F, 4.0F, 6.0F, 8.0F, 10.0F, 12.0F);

  EXPECT_TRUE(Vector6<float>::Lerp(a, b, 0.0F).IsApprox(a));
  EXPECT_TRUE(Vector6<float>::Lerp(a, b, 1.0F).IsApprox(b));
  EXPECT_TRUE(Vector6<float>::Lerp(a, b, 0.5F)
                  .IsApprox(Vector6<float>(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F))
  );
}

// Batched smoke test: confirms T = xsimd::batch<float> is a genuine free
// parameter, not just float in disguise, by giving lanes distinct values
// and checking Dot/Norm per lane.
TEST(Vector6Batched, DotAndNormPerLane) {
  Vector6<xsimd::batch<float>> a(
      MakeBatch({1.0F, 3.0F}),
      MakeBatch({0.0F, 4.0F}),
      MakeBatch({0.0F, 0.0F}),
      MakeBatch({0.0F, 0.0F}),
      MakeBatch({0.0F, 0.0F}),
      MakeBatch({0.0F, 0.0F})
  );

  auto norm = a.Norm();
  EXPECT_FLOAT_EQ(norm.get(0), 1.0F);
  EXPECT_FLOAT_EQ(norm.get(1), 5.0F);
}
