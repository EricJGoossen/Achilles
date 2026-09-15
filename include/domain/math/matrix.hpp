#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <initializer_list>
#include <tuple>
#include <type_traits>
#include <utility>
#include <xsimd/xsimd.hpp>

#include "activation_mask.hpp"
#include "engine/assembler.hpp"
#include "util/simd_ops.hpp"

namespace achilles::domain::math {

template <
    util::ArithmeticLike T,
    std::size_t R,
    std::size_t C,
    bool IsConst = false,
    bool Transposed = false>
class MatrixBlock;

template <util::ArithmeticLike T, std::size_t M, std::size_t N>
class Matrix {
  using Mask = decltype(std::declval<T>() == std::declval<T>());

 public:
  using ScalarType = T;

  Matrix() : data_() {};
  explicit Matrix(const std::array<T, M * N>& data) : data_(data) {}
  template <typename... Args>
  constexpr explicit Matrix(Args... args) : data_{static_cast<T>(args)...} {
    static_assert(
        sizeof...(Args) == M * N, "Wrong number of elements for matrix size"
    );
    static_assert(
        (std::is_convertible_v<Args, T> && ...),
        "All arguments must be convertible to T"
    );
  }

  static constexpr Matrix Zero() { return {}; }
  constexpr Matrix& SetZero() {
    data_ = {};
    return *this;
  }
  static constexpr Matrix Ones() {
    Matrix result;
    result.data_.fill(T{1});
    return result;
  }
  constexpr Matrix& SetOnes() {
    data_.fill(T{1});
    return *this;
  }
  static constexpr Matrix Identity() {
    static_assert(M == N, "Identity only defined for square matrices");
    Matrix result;
    for (std::size_t i = 0; i < M; ++i) {
      result(i, i) = T{1};
    }
    return result;
  }
  constexpr Matrix& SetIdentity() {
    static_assert(M == N, "SetIdentity is only defined for square matrices");
    SetZero();
    for (std::size_t i = 0; i < M; ++i) {
      (*this)(i, i) = T{1};
    }
    return *this;
  }

  // Access
  constexpr auto ToTuple() const {
    return ToTupleImpl(std::make_index_sequence<M * N>{});
  }
  constexpr std::size_t Rows() const { return M; }
  constexpr std::size_t Cols() const { return N; }
  constexpr T& operator()(std::size_t i, std::size_t j) {
    assert(i < M && j < N && "Matrix indices out of bounds");
    return data_[i * N + j];
  }
  constexpr const T& operator()(std::size_t i, std::size_t j) const {
    assert(i < M && j < N && "Matrix indices out of bounds");
    return data_[i * N + j];
  }
  constexpr T& operator[](std::size_t i) {
    assert(i < M * N && "Matrix index out of bounds");
    return data_[i];
  }
  constexpr const T& operator[](std::size_t i) const {
    assert(i < M * N && "Matrix index out of bounds");
    return data_[i];
  }
  constexpr Matrix<T, 1, N> Row(std::size_t i) const {
    assert(i < M && "Row index out of bounds");

    std::array<T, N> result;
    std::copy(
        data_.begin() + i * N, data_.begin() + (i + 1) * N, result.begin()
    );
    return Matrix<T, 1, N>(result);
  }
  constexpr Matrix<T, M, 1> Column(std::size_t j) const {
    assert(j < N && "Column index out of bounds");

    std::array<T, M> result;
    for (std::size_t i = 0; i < M; ++i) {
      result[i] = (*this)(i, j);
    }
    return Matrix<T, M, 1>(result);
  }
  template <std::size_t R, std::size_t C>
  constexpr Matrix<T, R, C> Submatrix(
      std::size_t row_start, std::size_t col_start
  ) const {
    static_assert(
        R <= M && C <= N,
        "Submatrix dimensions must be less than or equal to original "
        "matrix dimensions"
    );
    assert(
        row_start + R <= M && col_start + C <= N &&
        "Submatrix indices out of bounds"
    );
    Matrix<T, R, C> result;
    for (std::size_t i = 0; i < R; ++i) {
      for (std::size_t j = 0; j < C; ++j) {
        result(i, j) = (*this)(row_start + i, col_start + j);
      }
    }
    return result;
  }
  template <std::size_t R, std::size_t C>
  constexpr void SetSubmatrix(
      std::size_t row_start,
      std::size_t col_start,
      const Matrix<T, R, C>& submatrix
  ) {
    static_assert(
        R <= M && C <= N,
        "Submatrix dimensions must be less than or equal to original "
        "matrix dimensions"
    );
    assert(
        row_start + R <= M && col_start + C <= N &&
        "Submatrix indices out of bounds"
    );
    for (std::size_t i = 0; i < R; ++i) {
      for (std::size_t j = 0; j < C; ++j) {
        (*this)(row_start + i, col_start + j) = submatrix(i, j);
      }
    }
  }
  template <std::size_t R, std::size_t C>
  constexpr MatrixBlock<T, R, C> Block(
      std::size_t row_start, std::size_t col_start
  ) {
    assert(row_start + R <= M && col_start + C <= N);
    return MatrixBlock<T, R, C>(data_.data(), N, row_start, col_start);
  }

  template <std::size_t R, std::size_t C>
  constexpr MatrixBlock<T, R, C, true> Block(
      std::size_t row_start, std::size_t col_start
  ) const {
    assert(row_start + R <= M && col_start + C <= N);
    return MatrixBlock<T, R, C, true>(data_.data(), N, row_start, col_start);
  }

  // Comparison
  friend constexpr Mask operator==(const Matrix& a, const Matrix& b) {
    Mask result = (a.data_[0] == b.data_[0]);
    for (std::size_t i = 1; i < M * N; ++i) {
      result = result & (a.data_[i] == b.data_[i]);
    }
    return result;
  }
  friend constexpr Mask operator!=(const Matrix& a, const Matrix& b) {
    return !(a == b);
  }
  constexpr Mask IsApprox(const Matrix& other, float epsilon = 1e-5F) const {
    using std::abs;
    using xsimd::abs;

    Mask result = abs(data_[0] - other[0]) <= epsilon;
    for (std::size_t i = 1; i < M * N; ++i) {
      result = result & (abs(data_[i] - other[i]) <= epsilon);
    }
    return result;
  }
  constexpr Mask IsZero(float epsilon = 1e-8F) const {
    return this->IsApprox(Matrix::Zero(), epsilon);
  }

  // Conversion
  template <typename U>
  constexpr Matrix<U, M, N> Cast() const {
    Matrix<U, M, N> result;
    for (std::size_t i = 0; i < M * N; ++i) {
      result[i] = static_cast<U>(data_[i]);
    }
    return result;
  }

  // Elementwise Arithmetic
  constexpr Matrix operator+(const Matrix& other) const {
    Matrix result;
    for (std::size_t i = 0; i < M * N; ++i) {
      result[i] = data_[i] + other[i];
    }
    return result;
  }
  constexpr Matrix operator-(const Matrix& other) const {
    Matrix result;
    for (std::size_t i = 0; i < M * N; ++i) {
      result[i] = data_[i] - other[i];
    }
    return result;
  }
  constexpr Matrix operator-() const {
    Matrix result = *this;
    return result.NegateInPlace();
  }
  constexpr Matrix& NegateInPlace() {
    for (std::size_t i = 0; i < M * N; ++i) {
      data_[i] = -data_[i];
    }
    return *this;
  }
  constexpr Matrix& operator+=(const Matrix& other) {
    for (std::size_t i = 0; i < M * N; ++i) {
      data_[i] += other[i];
    }
    return *this;
  }
  constexpr Matrix& operator-=(const Matrix& other) {
    for (std::size_t i = 0; i < M * N; ++i) {
      data_[i] -= other[i];
    }
    return *this;
  }

  // Scalar Algebra
  constexpr Matrix operator*(T scalar) const {
    Matrix result;
    for (std::size_t i = 0; i < M * N; ++i) {
      result[i] = data_[i] * scalar;
    }
    return result;
  }
  friend Matrix operator*(T scalar, const Matrix& m) { return m * scalar; }
  constexpr Matrix operator/(T scalar) const {
    Matrix result;
    for (std::size_t i = 0; i < M * N; ++i) {
      result[i] = data_[i] / scalar;
    }
    return result;
  }
  constexpr Matrix& operator*=(T scalar) {
    for (std::size_t i = 0; i < M * N; ++i) {
      data_[i] *= scalar;
    }
    return *this;
  }
  constexpr Matrix& operator/=(T scalar) {
    for (std::size_t i = 0; i < M * N; ++i) {
      data_[i] /= scalar;
    }
    return *this;
  }

  // Matrix Operations
  template <std::size_t P>
  constexpr Matrix<T, M, P> operator*(const Matrix<T, N, P>& other) const {
    Matrix<T, M, P> result;
    for (std::size_t i = 0; i < M; ++i) {
      for (std::size_t j = 0; j < P; ++j) {
        T sum = T{0};
        for (std::size_t k = 0; k < N; ++k) {
          sum += (*this)(i, k) * other(k, j);
        }
        result(i, j) = sum;
      }
    }
    return result;
  }
  constexpr Matrix<T, N, M> Transpose() const {
    Matrix<T, N, M> result;
    for (std::size_t i = 0; i < M; ++i) {
      for (std::size_t j = 0; j < N; ++j) {
        result(j, i) = (*this)(i, j);
      }
    }
    return result;
  }
  constexpr Matrix& TransposeInPlace() {
    static_assert(
        M == N,
        "TransposeInPlace is only defined for square matrices; use "
        "Transpose() for non-square"
    );
    for (std::size_t i = 0; i < M; ++i) {
      for (std::size_t j = i + 1; j < N; ++j) {
        T tmp = (*this)(i, j);
        (*this)(i, j) = (*this)(j, i);
        (*this)(j, i) = tmp;
      }
    }
    return *this;
  }
  constexpr T NormSquared() const {
    T sum = T{0};
    for (std::size_t i = 0; i < M * N; ++i) {
      sum += data_[i] * data_[i];
    }
    return sum;
  }
  constexpr T Norm() const {
    using std::sqrt;
    using xsimd::sqrt;
    return sqrt(NormSquared());
  }
  constexpr Matrix Normalize() const {
    Matrix result = *this;
    return result.NormalizeInPlace();
  }
  constexpr Matrix& NormalizeInPlace() {
    T norm = Norm();
    assert(!util::AnyTrue(norm < T{1e-9}) && "Cannot normalize a zero vector");
    for (std::size_t i = 0; i < M * N; ++i) {
      data_[i] /= norm;
    }
    return *this;
  }
  constexpr T Determinant() const {
    static_assert(M == N, "Determinant is only defined for square matrices");
    static_assert(
        M == 2 || M == 3 || M == 6,
        "Determinant is only implemented for 2x2, 3x3, and 6x6 matrices"
    );
    if constexpr (M == 2) {
      return (*this)(0, 0) * (*this)(1, 1) - (*this)(0, 1) * (*this)(1, 0);
    } else if constexpr (M == 3) {
      return (*this)(0, 0) * ((*this)(1, 1) * (*this)(2, 2) -
                              (*this)(1, 2) * (*this)(2, 1)) -
             (*this)(0, 1) * ((*this)(1, 0) * (*this)(2, 2) -
                              (*this)(1, 2) * (*this)(2, 0)) +
             (*this)(0, 2) * ((*this)(1, 0) * (*this)(2, 1) -
                              (*this)(1, 1) * (*this)(2, 0));
    } else if constexpr (M == 6) {
      // Use block matrix determinant formula for 6x6 matrices. This Schur-
      // complement approach requires the top-left 3x3 block to be
      // invertible, which does not hold for an arbitrary 6x6 matrix -- it
      // is only valid here because Matrix<T, 6, 6> is used exclusively for
      // spatial-inertia-like operators, whose top-left (rotational
      // inertia) block is guaranteed invertible whenever the matrix itself
      // is a physically valid, symmetric positive definite inertia
      // operator (every principal submatrix of an SPD matrix is
      // invertible).
      Matrix<T, 3, 3> A = Submatrix<3, 3>(0, 0);
      Matrix<T, 3, 3> B = Submatrix<3, 3>(0, 3);
      Matrix<T, 3, 3> C = Submatrix<3, 3>(3, 0);
      Matrix<T, 3, 3> D = Submatrix<3, 3>(3, 3);

      T detA = A.Determinant();
      assert(
          util::AllTrue(IsNonSingularDeterminant(detA)) &&
          "Determinant (6x6): top-left 3x3 block must be invertible -- "
          "only defined for spatial-inertia-like matrices"
      );

      Matrix<T, 3, 3> schur = D - C * A.InverseInPlace() * B;
      return detA * schur.Determinant();
    }
  }

  // Square Matrix Operations
  constexpr T Trace() const {
    static_assert(M == N, "Trace is only defined for square matrices");
    T sum = T{0};
    for (std::size_t i = 0; i < M; ++i) {
      sum += (*this)(i, i);
    }
    return sum;
  }
  constexpr Matrix& InverseInPlace() {
    static_assert(M == N, "InverseInPlace is only defined for square matrices");
    static_assert(
        M == 2 || M == 3 || M == 6,
        "InverseInPlace is only implemented for 2x2, 3x3, and 6x6 "
        "matrices"
    );
    if constexpr (M == 2) {
      return Inverse2x2InPlace();
    } else if constexpr (M == 3) {
      return Inverse3x3InPlace();
    } else if constexpr (M == 6) {
      return Inverse6x6InPlace();
    }
  }
  constexpr Matrix Inverse() const {
    Matrix result = *this;
    result.InverseInPlace();
    return result;
  }

  // Vector Operations
  template <std::size_t P>
  constexpr Matrix<T, M, P> OuterProduct(const Matrix<T, P, 1>& other) const {
    static_assert(N == 1, "Outer product is only defined for vectors");
    Matrix<T, M, P> result;
    for (std::size_t i = 0; i < M; ++i) {
      for (std::size_t j = 0; j < P; ++j) {
        result(i, j) = data_[i] * other[j];
      }
    }
    return result;
  }

  constexpr T Dot(const Matrix& other) const {
    static_assert(M == 1 || N == 1, "Dot product is only defined for vectors");
    T sum = T{0};
    for (std::size_t i = 0; i < M * N; ++i) {
      sum += data_[i] * other[i];
    }
    return sum;
  }

  // Inverse helper functions
  constexpr Matrix& Inverse2x2InPlace() {
    static_assert(
        M == 2 && N == 2, "Inverse2x2InPlace is only defined for 2x2 matrices"
    );

    T a = (*this)(0, 0);
    T b = (*this)(0, 1);
    T c = (*this)(1, 0);
    T d = (*this)(1, 1);

    T inv_det = T{1} / (a * d - b * c);

    (*this)(0, 0) = d * inv_det;
    (*this)(0, 1) = -b * inv_det;
    (*this)(1, 0) = -c * inv_det;
    (*this)(1, 1) = a * inv_det;

    return *this;
  }
  constexpr Matrix& Inverse3x3InPlace() {
    static_assert(
        M == 3 && N == 3, "Inverse3x3InPlace is only defined for 3x3 matrices"
    );

    T a = (*this)(0, 0);
    T b = (*this)(0, 1);
    T c = (*this)(0, 2);
    T d = (*this)(1, 0);
    T e = (*this)(1, 1);
    T f = (*this)(1, 2);
    T g = (*this)(2, 0);
    T h = (*this)(2, 1);
    T i = (*this)(2, 2);

    T c00 = e * i - f * h;
    T c01 = c * h - b * i;
    T c02 = b * f - c * e;
    T c10 = f * g - d * i;
    T c11 = a * i - c * g;
    T c12 = c * d - a * f;
    T c20 = d * h - e * g;
    T c21 = b * g - a * h;
    T c22 = a * e - b * d;

    T inv_det = T{1} / (a * c00 + b * c10 + c * c20);

    (*this)(0, 0) = c00 * inv_det;
    (*this)(0, 1) = c01 * inv_det;
    (*this)(0, 2) = c02 * inv_det;
    (*this)(1, 0) = c10 * inv_det;
    (*this)(1, 1) = c11 * inv_det;
    (*this)(1, 2) = c12 * inv_det;
    (*this)(2, 0) = c20 * inv_det;
    (*this)(2, 1) = c21 * inv_det;
    (*this)(2, 2) = c22 * inv_det;

    return *this;
  }
  constexpr Matrix& Inverse6x6InPlace() {
    static_assert(
        M == 6 && N == 6, "Inverse6x6InPlace is only defined for 6x6 matrices"
    );

    // Like Determinant()'s M == 6 branch, this Schur-complement inversion
    // requires the top-left 3x3 block (P) and the Schur complement itself
    // to be invertible -- true whenever this matrix is a physically valid
    // spatial-inertia-like SPD operator (the only thing Matrix<T, 6, 6> is
    // used for in this codebase), but not guaranteed for an arbitrary 6x6
    // matrix.
    Matrix<T, 3, 3> p = Submatrix<3, 3>(0, 0);
    Matrix<T, 3, 3> q = Submatrix<3, 3>(0, 3);
    Matrix<T, 3, 3> r = Submatrix<3, 3>(3, 0);
    Matrix<T, 3, 3> s = Submatrix<3, 3>(3, 3);

    assert(
        util::AllTrue(IsNonSingularDeterminant(p.Determinant())) &&
        "Inverse6x6InPlace: top-left 3x3 block (P) must be invertible -- "
        "only defined for spatial-inertia-like matrices"
    );

    Matrix<T, 3, 3> p_inv = p.Inverse();
    Matrix<T, 3, 3> sigma = s - r * p_inv * q;

    assert(
        util::AllTrue(IsNonSingularDeterminant(sigma.Determinant())) &&
        "Inverse6x6InPlace: Schur complement (S - R*P^-1*Q) must be "
        "invertible"
    );

    Matrix<T, 3, 3> sigma_inv = sigma.Inverse();
    Matrix<T, 3, 3> top_left = p_inv + p_inv * q * sigma_inv * r * p_inv;
    Matrix<T, 3, 3> top_right = -p_inv * q * sigma_inv;
    Matrix<T, 3, 3> bottom_left = -sigma_inv * r * p_inv;
    Matrix<T, 3, 3> bottom_right = sigma_inv;

    SetSubmatrix(0, 0, top_left);
    SetSubmatrix(0, 3, top_right);
    SetSubmatrix(3, 0, bottom_left);
    SetSubmatrix(3, 3, bottom_right);

    return *this;
  }

  // Binary Operations
  template <MaskLike MaskT>
    requires(MaskT::Size() == M)
  constexpr Matrix& ApplyMaskRowsInPlace(MaskT m) {
    for (std::size_t i = 0; i < M; ++i) {
      for (std::size_t j = 0; j < N; ++j) {
        (*this)(i, j) = util::Select(m[i], (*this)(i, j), T{0});
      }
    }
    return *this;
  }
  template <MaskLike MaskT>
    requires(MaskT::Size() == M)
  constexpr Matrix ApplyMaskRows(MaskT m) const {
    Matrix result = *this;
    return result.ApplyMaskRowsInPlace(m);
  }
  template <MaskLike MaskT>
    requires(MaskT::Size() == N)
  constexpr Matrix& ApplyMaskColsInPlace(MaskT m) {
    for (std::size_t j = 0; j < N; ++j) {
      for (std::size_t i = 0; i < M; ++i) {
        (*this)(i, j) = util::Select(m[j], (*this)(i, j), T{0});
      }
    }
    return *this;
  }
  template <MaskLike MaskT>
    requires(MaskT::Size() == N)
  constexpr Matrix ApplyMaskCols(MaskT m) const {
    Matrix result = *this;
    return result.ApplyMaskColsInPlace(m);
  }

  // Mask Operations
  template <MaskLike MaskT>
    requires(MaskT::Size() == M && M == N)
  constexpr Matrix& MaskedInverseInPlace(MaskT m) {
    // Substitute the identity on the diagonal for inactive rows/columns
    // rather than zeroing them outright, which would leave the matrix
    // singular and unable to be inverted.
    for (std::size_t i = 0; i < M; ++i) {
      for (std::size_t j = 0; j < M; ++j) {
        if (j == i) {
          continue;
        }
        (*this)(i, j) = util::Select(m[i], (*this)(i, j), T{0});
        (*this)(j, i) = util::Select(m[i], (*this)(j, i), T{0});
      }
      (*this)(i, i) = util::Select(m[i], (*this)(i, i), T{1});
    }
    return InverseInPlace();
  }
  template <MaskLike MaskT>
    requires(MaskT::Size() == M && M == N)
  constexpr Matrix MaskedInverse(MaskT m) const {
    Matrix result = *this;
    return result.MaskedInverseInPlace(m);
  }

  // Printing
  friend std::ostream& operator<<(
      std::ostream& os, const math::Matrix<T, M, N>& m
  ) {
    os << "Matrix" << M << "x" << N << "(";
    for (std::size_t i = 0; i < M; ++i) {
      os << "(";
      for (std::size_t j = 0; j < N; ++j) {
        os << m(i, j);
        if (j < N - 1) {
          os << ", ";
        }
      }
      os << ")";
      if (i < M - 1) {
        os << ", ";
      }
    }
    os << ")";
    return os;
  }

 private:
  template <std::size_t... Is>
  constexpr auto ToTupleImpl(std::index_sequence<Is...>) const {
    return std::make_tuple(data_[Is]...);
  }

  // Shared precondition check for the 6x6 Schur-complement Determinant()/
  // Inverse6x6InPlace() blocks: a 3x3 block's determinant must be
  // meaningfully nonzero for that block to be safely inverted/used as a
  // pivot.
  static constexpr Mask IsNonSingularDeterminant(T det) {
    using std::abs;
    using xsimd::abs;
    return abs(det) >= T{1e-9};
  }

  std::array<T, M * N> data_{};
};

template <
    util::ArithmeticLike T,
    std::size_t R,
    std::size_t C,
    bool IsConst,
    bool Transposed>
class MatrixBlock {
  using DataPtr = std::conditional_t<IsConst, const T*, T*>;
  using ElemRef = std::conditional_t<IsConst, const T&, T&>;

 public:
  constexpr MatrixBlock(
      DataPtr data,
      std::size_t stride,
      std::size_t row_start,
      std::size_t col_start
  )
      : data_(data),
        stride_(stride),
        row_start_(row_start),
        col_start_(col_start) {}

  constexpr ElemRef operator()(std::size_t i, std::size_t j) const {
    if constexpr (Transposed) {
      return data_[(row_start_ + j) * stride_ + (col_start_ + i)];
    } else {
      return data_[(row_start_ + i) * stride_ + (col_start_ + j)];
    }
  }

  constexpr MatrixBlock<T, C, R, IsConst, !Transposed> Transpose() const {
    return MatrixBlock<T, C, R, IsConst, !Transposed>(
        data_, stride_, row_start_, col_start_
    );
  }

  template <std::size_t P>
  constexpr Matrix<T, R, P> operator*(const Matrix<T, C, P>& other) const {
    Matrix<T, R, P> result;
    for (std::size_t i = 0; i < R; ++i) {
      for (std::size_t j = 0; j < P; ++j) {
        T sum = T{0};
        for (std::size_t k = 0; k < C; ++k) {
          sum += (*this)(i, k) * other(k, j);
        }
        result(i, j) = sum;
      }
    }
    return result;
  }

  constexpr MatrixBlock& operator=(const Matrix<T, R, C>& rhs) {
    for (std::size_t i = 0; i < R; ++i) {
      for (std::size_t j = 0; j < C; ++j) {
        (*this)(i, j) = rhs(i, j);
      }
    }
    return *this;
  }

 private:
  DataPtr data_;
  std::size_t stride_;
  std::size_t row_start_, col_start_;
};
template <typename T>
using Matrix6x6 = Matrix<T, 6, 6>;
template <typename T>
using Matrix6x6Assembler =
    engine::Assembler<Matrix6x6, T, engine::RepeatTypes<36>>;
static_assert(engine::AssemblerLike<Matrix6x6Assembler<float>>);

template <typename T>
using Matrix3x3 = Matrix<T, 3, 3>;
template <typename T>
using Matrix3x3Assembler =
    engine::Assembler<Matrix3x3, T, engine::RepeatTypes<9>>;
static_assert(engine::AssemblerLike<Matrix3x3Assembler<float>>);

template <typename T>
using Matrix2x2 = Matrix<T, 2, 2>;
template <typename T>
using Matrix2x2Assembler =
    engine::Assembler<Matrix2x2, T, engine::RepeatTypes<4>>;
static_assert(engine::AssemblerLike<Matrix2x2Assembler<float>>);

}  // namespace achilles::domain::math