#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "domain/math/activation_mask.hpp"
#include "domain/math/matrix.hpp"
#include "support/mask_archetype.hpp"

using achilles::domain::math::ActivationMask;
using achilles::domain::math::Matrix;
using achilles::test_support::MaskArchetype;

namespace {

// Diagonal 6x6 with distinct, well-conditioned entries: keeps the
// Determinant/InverseInPlace block-formula tests (which both require the
// top-left 3x3 block, and its Schur complement, to be nonsingular) free
// of hand-computed cofactor arithmetic.
Matrix<float, 6, 6> DiagonalMatrix(std::array<float, 6> diag) {
  Matrix<float, 6, 6> m = Matrix<float, 6, 6>::Zero();
  for (std::size_t i = 0; i < 6; ++i) {
    m(i, i) = diag.at(i);
  }
  return m;
}

}  // namespace

TEST(MatrixConstruction, DefaultIsZero) {
  Matrix<float, 2, 2> m;
  EXPECT_TRUE(m.IsZero());
}

TEST(MatrixConstruction, FromArray) {
  Matrix<float, 2, 2> m(std::array<float, 4>{1.0F, 2.0F, 3.0F, 4.0F});
  EXPECT_FLOAT_EQ(m(0, 0), 1.0F);
  EXPECT_FLOAT_EQ(m(0, 1), 2.0F);
  EXPECT_FLOAT_EQ(m(1, 0), 3.0F);
  EXPECT_FLOAT_EQ(m(1, 1), 4.0F);
}

TEST(MatrixConstruction, FromVariadicArgsIsRowMajor) {
  Matrix<float, 2, 2> m(1.0F, 2.0F, 3.0F, 4.0F);
  EXPECT_FLOAT_EQ(m(0, 0), 1.0F);
  EXPECT_FLOAT_EQ(m(0, 1), 2.0F);
  EXPECT_FLOAT_EQ(m(1, 0), 3.0F);
  EXPECT_FLOAT_EQ(m(1, 1), 4.0F);
}

TEST(MatrixStaticConstructors, ZeroOnesIdentity) {
  EXPECT_TRUE((Matrix<float, 3, 3>::Zero().IsZero()));

  Matrix<float, 2, 3> ones = Matrix<float, 2, 3>::Ones();
  for (std::size_t i = 0; i < 2; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      EXPECT_FLOAT_EQ(ones(i, j), 1.0F);
    }
  }

  Matrix<float, 3, 3> id = Matrix<float, 3, 3>::Identity();
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      EXPECT_FLOAT_EQ(id(i, j), i == j ? 1.0F : 0.0F);
    }
  }
}

TEST(MatrixStaticConstructors, SetZeroSetOnesSetIdentityInPlace) {
  Matrix<float, 2, 2> m(1.0F, 2.0F, 3.0F, 4.0F);

  Matrix<float, 2, 2> a = m;
  a.SetZero();
  EXPECT_TRUE(a.IsZero());

  Matrix<float, 2, 2> b = m;
  b.SetOnes();
  EXPECT_TRUE(b.IsApprox(Matrix<float, 2, 2>::Ones()));

  Matrix<float, 2, 2> c = m;
  c.SetIdentity();
  EXPECT_TRUE(c.IsApprox(Matrix<float, 2, 2>::Identity()));
}

// Access: ToTuple, Rows/Cols, operator()(i,j) (mutable and const),
// operator[] (mutable and const) must all agree on the same data.
TEST(MatrixAccess, ElementAccessorsAgree) {
  Matrix<float, 2, 2> m(1.0F, 2.0F, 3.0F, 4.0F);

  EXPECT_EQ(m.Rows(), 2U);
  EXPECT_EQ(m.Cols(), 2U);
  EXPECT_FLOAT_EQ(m[0], 1.0F);
  EXPECT_FLOAT_EQ(m[3], 4.0F);
  EXPECT_FLOAT_EQ(m(1, 1), m[3]);

  const Matrix<float, 2, 2>& const_ref = m;
  EXPECT_FLOAT_EQ(const_ref(0, 1), 2.0F);
  EXPECT_FLOAT_EQ(const_ref[2], 3.0F);

  m(0, 0) = 9.0F;
  EXPECT_FLOAT_EQ(m[0], 9.0F);
  m[1] = 8.0F;
  EXPECT_FLOAT_EQ(m(0, 1), 8.0F);

  auto tuple = m.ToTuple();
  EXPECT_FLOAT_EQ(std::get<0>(tuple), m[0]);
  EXPECT_FLOAT_EQ(std::get<3>(tuple), m[3]);
}

TEST(MatrixAccess, RowAndColumn) {
  Matrix<float, 2, 3> m(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);

  Matrix<float, 1, 3> row0 = m.Row(0);
  EXPECT_FLOAT_EQ(row0(0, 0), 1.0F);
  EXPECT_FLOAT_EQ(row0(0, 2), 3.0F);

  Matrix<float, 2, 1> col1 = m.Column(1);
  EXPECT_FLOAT_EQ(col1(0, 0), 2.0F);
  EXPECT_FLOAT_EQ(col1(1, 0), 5.0F);
}

TEST(MatrixAccess, SubmatrixAndSetSubmatrix) {
  Matrix<float, 3, 3> m(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F, 8.0F, 9.0F);

  Matrix<float, 2, 2> sub = m.Submatrix<2, 2>(1, 1);
  EXPECT_FLOAT_EQ(sub(0, 0), 5.0F);
  EXPECT_FLOAT_EQ(sub(0, 1), 6.0F);
  EXPECT_FLOAT_EQ(sub(1, 0), 8.0F);
  EXPECT_FLOAT_EQ(sub(1, 1), 9.0F);

  Matrix<float, 2, 2> zeros = Matrix<float, 2, 2>::Zero();
  m.SetSubmatrix(0, 0, zeros);
  EXPECT_FLOAT_EQ(m(0, 0), 0.0F);
  EXPECT_FLOAT_EQ(m(0, 1), 0.0F);
  EXPECT_FLOAT_EQ(m(1, 0), 0.0F);
  EXPECT_FLOAT_EQ(m(1, 1), 0.0F);
  EXPECT_FLOAT_EQ(m(2, 2), 9.0F);  // untouched
}

// Block()/const Block() are the mutating and read-only views onto the same
// backing storage: writing through a mutable Block must be visible both
// through the original matrix and through a freshly taken const Block.
TEST(MatrixAccess, BlockViewsAliasOriginalStorage) {
  Matrix<float, 3, 3> m = Matrix<float, 3, 3>::Zero();
  auto block = m.Block<2, 2>(0, 0);
  block(0, 0) = 1.0F;
  block(1, 1) = 2.0F;

  EXPECT_FLOAT_EQ(m(0, 0), 1.0F);
  EXPECT_FLOAT_EQ(m(1, 1), 2.0F);

  const Matrix<float, 3, 3>& const_m = m;
  auto const_block = const_m.Block<2, 2>(0, 0);
  EXPECT_FLOAT_EQ(const_block(0, 0), 1.0F);
  EXPECT_FLOAT_EQ(const_block(1, 1), 2.0F);
}

TEST(MatrixComparison, EqualityAndInequality) {
  Matrix<float, 2, 2> a(1.0F, 2.0F, 3.0F, 4.0F);
  Matrix<float, 2, 2> b = a;
  Matrix<float, 2, 2> c(1.0F, 2.0F, 3.0F, 4.5F);

  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a != b);
  EXPECT_TRUE(a != c);
  EXPECT_FALSE(a == c);
}

TEST(MatrixComparison, IsApproxWithinAndOutsideEpsilon) {
  Matrix<float, 2, 2> a(1.0F, 2.0F, 3.0F, 4.0F);
  Matrix<float, 2, 2> close = a;
  close(0, 0) += 1e-7F;
  Matrix<float, 2, 2> far = a;
  far(0, 0) += 0.1F;

  EXPECT_TRUE(a.IsApprox(close));
  EXPECT_FALSE(a.IsApprox(far));
}

TEST(MatrixComparison, IsZero) {
  EXPECT_TRUE((Matrix<float, 2, 2>::Zero().IsZero()));
  EXPECT_FALSE((Matrix<float, 2, 2>::Identity().IsZero()));
}

TEST(MatrixConversion, CastToDifferentScalarType) {
  Matrix<float, 2, 2> m(1.7F, 2.2F, -3.9F, 4.4F);
  Matrix<int, 2, 2> casted = m.Cast<int>();
  EXPECT_EQ(casted(0, 0), static_cast<int>(1.7F));
  EXPECT_EQ(casted(1, 0), static_cast<int>(-3.9F));
}

// Elementwise arithmetic: +, -, unary -, NegateInPlace, +=, -=.
TEST(MatrixArithmetic, AdditionAndSubtraction) {
  Matrix<float, 2, 2> a(1.0F, 2.0F, 3.0F, 4.0F);
  Matrix<float, 2, 2> b(4.0F, 3.0F, 2.0F, 1.0F);

  Matrix<float, 2, 2> sum = a + b;
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_FLOAT_EQ(sum[i], 5.0F);
  }

  Matrix<float, 2, 2> diff = a - b;
  EXPECT_FLOAT_EQ(diff[0], -3.0F);
  EXPECT_FLOAT_EQ(diff[3], 3.0F);
}

TEST(MatrixArithmetic, UnaryNegationAndNegateInPlace) {
  Matrix<float, 2, 2> a(1.0F, -2.0F, 3.0F, -4.0F);

  Matrix<float, 2, 2> negated = -a;
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_FLOAT_EQ(negated[i], -a[i]);
  }

  Matrix<float, 2, 2> b = a;
  b.NegateInPlace();
  EXPECT_TRUE(b.IsApprox(negated));
}

TEST(MatrixArithmetic, CompoundAddAndSubtract) {
  Matrix<float, 2, 2> a(1.0F, 2.0F, 3.0F, 4.0F);
  Matrix<float, 2, 2> b = Matrix<float, 2, 2>::Ones();

  Matrix<float, 2, 2> c = a;
  c += b;
  EXPECT_TRUE(c.IsApprox(a + b));
  c -= b;
  EXPECT_TRUE(c.IsApprox(a));
}

// Scalar algebra: *, /, *=, /=, and the friend scalar*matrix overload.
TEST(MatrixScalarAlgebra, MultiplyAndDivide) {
  Matrix<float, 2, 2> m(2.0F, 4.0F, 6.0F, 8.0F);

  Matrix<float, 2, 2> scaled = m * 0.5F;
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_FLOAT_EQ(scaled[i], m[i] * 0.5F);
  }

  Matrix<float, 2, 2> divided = m / 2.0F;
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_FLOAT_EQ(divided[i], m[i] / 2.0F);
  }

  EXPECT_TRUE((3.0F * m).IsApprox(m * 3.0F));
}

TEST(MatrixScalarAlgebra, CompoundMultiplyAndDivide) {
  Matrix<float, 2, 2> m = Matrix<float, 2, 2>::Ones();
  Matrix<float, 2, 2> a = m;
  a *= 4.0F;
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_FLOAT_EQ(a[i], 4.0F);
  }
  a /= 2.0F;
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_FLOAT_EQ(a[i], 2.0F);
  }
}

// Matrix operations: multiplication, transpose, norms.
TEST(MatrixOperations, MultiplicationByIdentityIsNoOp) {
  Matrix<float, 2, 3> m(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  EXPECT_TRUE((m * Matrix<float, 3, 3>::Identity()).IsApprox(m));
  EXPECT_TRUE((Matrix<float, 2, 2>::Identity() * m).IsApprox(m));
}

TEST(MatrixOperations, MultiplicationKnownResult) {
  Matrix<float, 2, 2> a(1.0F, 2.0F, 3.0F, 4.0F);
  Matrix<float, 2, 2> b(5.0F, 6.0F, 7.0F, 8.0F);
  // [1 2] [5 6]   [19 22]
  // [3 4] [7 8] = [43 50]
  Matrix<float, 2, 2> expected(19.0F, 22.0F, 43.0F, 50.0F);
  EXPECT_TRUE((a * b).IsApprox(expected));
}

TEST(MatrixOperations, TransposeAndTransposeInPlace) {
  Matrix<float, 2, 3> m(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  Matrix<float, 3, 2> transposed = m.Transpose();
  EXPECT_FLOAT_EQ(transposed(0, 0), 1.0F);
  EXPECT_FLOAT_EQ(transposed(2, 1), 6.0F);

  Matrix<float, 3, 3> square(
      1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F, 8.0F, 9.0F
  );
  Matrix<float, 3, 3> expected = square.Transpose();
  square.TransposeInPlace();
  EXPECT_TRUE(square.IsApprox(expected));
}

TEST(MatrixOperations, NormSquaredNormNormalize) {
  Matrix<float, 2, 1> v(3.0F, 4.0F);
  EXPECT_FLOAT_EQ(v.NormSquared(), 25.0F);
  EXPECT_FLOAT_EQ(v.Norm(), 5.0F);

  Matrix<float, 2, 1> unit = v.Normalize();
  EXPECT_FLOAT_EQ(unit.Norm(), 1.0F);

  Matrix<float, 2, 1> in_place = v;
  in_place.NormalizeInPlace();
  EXPECT_TRUE(in_place.IsApprox(unit));
}

TEST(MatrixOperations, Determinant2x2And3x3) {
  Matrix<float, 2, 2> m2(4.0F, 7.0F, 2.0F, 6.0F);
  EXPECT_FLOAT_EQ(m2.Determinant(), 10.0F);

  Matrix<float, 3, 3> m3(1.0F, 2.0F, 3.0F, 0.0F, 1.0F, 4.0F, 5.0F, 6.0F, 0.0F);
  EXPECT_FLOAT_EQ(m3.Determinant(), 1.0F);
}

TEST(MatrixOperations, Determinant6x6OfDiagonalIsProductOfDiagonal) {
  Matrix<float, 6, 6> m = DiagonalMatrix({1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F});
  EXPECT_FLOAT_EQ(m.Determinant(), 720.0F);
}

// The 6x6 Determinant()/InverseInPlace() block-Schur formula requires the
// top-left 3x3 block to be invertible (see DiagonalMatrix's comment
// above) -- true for the spatial-inertia-like matrices this type is
// actually used for, but not for an arbitrary 6x6 matrix. Zeroing that
// block out makes it singular; the precondition assert this used to
// never actually run (it sat behind an #ifdef on a macro nothing ever
// defined) must fire here instead of silently dividing by zero.
TEST(MatrixOperationsValidation, Determinant6x6WithSingularTopLeftBlockAsserts) {
  Matrix<float, 6, 6> m = Matrix<float, 6, 6>::Identity();
  m.SetSubmatrix(0, 0, Matrix<float, 3, 3>::Zero());
  EXPECT_DEATH(m.Determinant(), "top-left 3x3 block");
}

TEST(MatrixOperationsValidation, Inverse6x6WithSingularTopLeftBlockAsserts) {
  Matrix<float, 6, 6> m = Matrix<float, 6, 6>::Identity();
  m.SetSubmatrix(0, 0, Matrix<float, 3, 3>::Zero());
  EXPECT_DEATH(m.InverseInPlace(), "top-left 3x3 block");
}

// Inverse6x6InPlace has a second, separate precondition: the Schur
// complement (S - R*P^-1*Q) must itself be invertible, distinct from P
// (the top-left block, checked above). Zeroing only the bottom-right
// block leaves P == Identity (nonsingular, passes the first check) while
// the off-diagonal blocks stay zero, so the Schur complement reduces to
// S itself -- singular. Without this test the second assert would be
// exactly as unverified as the first one was before it was fixed.
TEST(MatrixOperationsValidation, Inverse6x6WithSingularSchurComplementAsserts) {
  Matrix<float, 6, 6> m = Matrix<float, 6, 6>::Identity();
  m.SetSubmatrix(3, 3, Matrix<float, 3, 3>::Zero());
  EXPECT_DEATH(m.InverseInPlace(), "Schur complement");
}

// Square matrix operations: Trace, Inverse/InverseInPlace for each
// implemented size (2x2, 3x3, 6x6), checked as a round trip (M * M^-1 ==
// I) rather than hand-derived inverse values.
TEST(MatrixSquareOperations, Trace) {
  Matrix<float, 3, 3> m(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F, 8.0F, 9.0F);
  EXPECT_FLOAT_EQ(m.Trace(), 1.0F + 5.0F + 9.0F);
}

TEST(MatrixSquareOperations, Inverse2x2RoundTrips) {
  Matrix<float, 2, 2> m(4.0F, 7.0F, 2.0F, 6.0F);
  EXPECT_TRUE((m * m.Inverse()).IsApprox(Matrix<float, 2, 2>::Identity(), 1e-4F)
  );
}

TEST(MatrixSquareOperations, Inverse3x3RoundTrips) {
  Matrix<float, 3, 3> m(1.0F, 2.0F, 3.0F, 0.0F, 1.0F, 4.0F, 5.0F, 6.0F, 0.0F);
  EXPECT_TRUE((m * m.Inverse()).IsApprox(Matrix<float, 3, 3>::Identity(), 1e-4F)
  );
}

TEST(MatrixSquareOperations, Inverse6x6RoundTrips) {
  Matrix<float, 6, 6> m = DiagonalMatrix({1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F});
  EXPECT_TRUE((m * m.Inverse()).IsApprox(Matrix<float, 6, 6>::Identity(), 1e-4F)
  );
}

TEST(MatrixSquareOperations, InverseInPlaceMatchesByValue) {
  Matrix<float, 2, 2> m(4.0F, 7.0F, 2.0F, 6.0F);
  Matrix<float, 2, 2> expected = m.Inverse();

  Matrix<float, 2, 2> in_place = m;
  in_place.InverseInPlace();
  EXPECT_TRUE(in_place.IsApprox(expected));
}

// Vector operations: OuterProduct, Dot. Only defined for N==1 (a column
// vector shape) per the method's own static_assert.
TEST(MatrixVectorOperations, OuterProduct) {
  Matrix<float, 2, 1> a(1.0F, 2.0F);
  Matrix<float, 3, 1> b(3.0F, 4.0F, 5.0F);
  Matrix<float, 2, 3> outer = a.OuterProduct(b);
  EXPECT_FLOAT_EQ(outer(0, 0), 3.0F);
  EXPECT_FLOAT_EQ(outer(0, 2), 5.0F);
  EXPECT_FLOAT_EQ(outer(1, 1), 8.0F);
}

TEST(MatrixVectorOperations, Dot) {
  Matrix<float, 3, 1> a(1.0F, 2.0F, 3.0F);
  Matrix<float, 3, 1> b(4.0F, -5.0F, 6.0F);
  EXPECT_FLOAT_EQ(a.Dot(b), 1.0F * 4.0F + 2.0F * -5.0F + 3.0F * 6.0F);
}

// Mask operations against the real ActivationMask -- the type these
// methods actually get called with in practice (see
// domain/math/activation_mask.hpp, which has its own full test file).
// The MaskArchetype tests below cover the same three methods again
// through a minimal MaskLike type, to prove they don't secretly depend on
// anything beyond the concept (ActivationMask's bit-packing, its
// ScalarType, ...).
TEST(MatrixMaskOperations, ApplyMaskRowsWithRealActivationMask) {
  Matrix<float, 3, 3> m(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F, 8.0F, 9.0F);
  ActivationMask<std::int32_t, 3> mask(true, false, true);

  Matrix<float, 3, 3> masked = m.ApplyMaskRows(mask);
  EXPECT_TRUE(masked.Row(0).IsApprox(m.Row(0)));
  EXPECT_TRUE(masked.Row(1).IsZero());
  EXPECT_TRUE(masked.Row(2).IsApprox(m.Row(2)));
}

// Mask operations, exercised through MaskArchetype (see above) to prove
// they only depend on the MaskLike interface.
TEST(MatrixMaskOperations, ApplyMaskRowsZeroesInactiveRows) {
  Matrix<float, 3, 3> m(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F, 8.0F, 9.0F);
  MaskArchetype<3> mask;
  mask.bits = {true, false, true};

  Matrix<float, 3, 3> masked = m.ApplyMaskRows(mask);
  EXPECT_TRUE(masked.Row(0).IsApprox(m.Row(0)));
  EXPECT_TRUE(masked.Row(1).IsZero());
  EXPECT_TRUE(masked.Row(2).IsApprox(m.Row(2)));

  Matrix<float, 3, 3> in_place = m;
  in_place.ApplyMaskRowsInPlace(mask);
  EXPECT_TRUE(in_place.IsApprox(masked));
}

TEST(MatrixMaskOperations, ApplyMaskColsZeroesInactiveColumns) {
  Matrix<float, 3, 3> m(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F, 8.0F, 9.0F);
  MaskArchetype<3> mask;
  mask.bits = {false, true, true};

  Matrix<float, 3, 3> masked = m.ApplyMaskCols(mask);
  EXPECT_TRUE(masked.Column(0).IsZero());
  EXPECT_TRUE(masked.Column(1).IsApprox(m.Column(1)));
  EXPECT_TRUE(masked.Column(2).IsApprox(m.Column(2)));

  Matrix<float, 3, 3> in_place = m;
  in_place.ApplyMaskColsInPlace(mask);
  EXPECT_TRUE(in_place.IsApprox(masked));
}

// MaskedInverse substitutes the identity on inactive diagonal entries
// instead of zeroing them, so the masked-out block of the result is
// exactly identity and the active block matches an ordinary inverse of
// just that block.
TEST(MatrixMaskOperations, MaskedInverseSubstitutesIdentityOnInactiveDiagonal) {
  Matrix<float, 3, 3> m = Matrix<float, 3, 3>::Zero();
  m(0, 0) = 2.0F;
  m(1, 1) = 4.0F;
  m(2, 2) = 5.0F;
  MaskArchetype<3> mask;
  mask.bits = {true, true, false};

  Matrix<float, 3, 3> masked_inv = m.MaskedInverse(mask);
  EXPECT_NEAR(masked_inv(0, 0), 0.5F, 1e-5F);
  EXPECT_NEAR(masked_inv(1, 1), 0.25F, 1e-5F);
  EXPECT_NEAR(masked_inv(2, 2), 1.0F, 1e-5F);  // inactive: identity, not 1/5

  Matrix<float, 3, 3> in_place = m;
  in_place.MaskedInverseInPlace(mask);
  EXPECT_TRUE(in_place.IsApprox(masked_inv));
}

// MatrixBlock: constructed only via Matrix::Block(); covers its own
// operator(), Transpose(), matrix multiplication and assignment-from-Matrix.
TEST(MatrixBlockOperations, TransposeSwapsDimensionsAndIndexing) {
  Matrix<float, 2, 3> m(1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F);
  auto block = m.Block<2, 3>(0, 0);
  auto transposed = block.Transpose();
  EXPECT_FLOAT_EQ(transposed(0, 0), block(0, 0));
  EXPECT_FLOAT_EQ(transposed(2, 1), block(1, 2));
}

TEST(MatrixBlockOperations, MultiplicationByMatrix) {
  Matrix<float, 2, 2> m(1.0F, 2.0F, 3.0F, 4.0F);
  auto block = m.Block<2, 2>(0, 0);
  Matrix<float, 2, 2> identity = Matrix<float, 2, 2>::Identity();
  EXPECT_TRUE((block * identity).IsApprox(m));
}

TEST(MatrixBlockOperations, AssignmentFromMatrixWritesThroughToOriginal) {
  Matrix<float, 3, 3> m = Matrix<float, 3, 3>::Zero();
  auto block = m.Block<2, 2>(1, 1);
  block = Matrix<float, 2, 2>(1.0F, 2.0F, 3.0F, 4.0F);

  EXPECT_FLOAT_EQ(m(1, 1), 1.0F);
  EXPECT_FLOAT_EQ(m(1, 2), 2.0F);
  EXPECT_FLOAT_EQ(m(2, 1), 3.0F);
  EXPECT_FLOAT_EQ(m(2, 2), 4.0F);
}
