#ifndef GRABCOMMON_LIBNUMERIC_MATRIX_H_
#define GRABCOMMON_LIBNUMERIC_MATRIX_H_

/**
 * Define whether the build target is a MCU or a device with a graphic interface.
 * @todo move this flag in the preprocessor flags
 */
#define MCU_TARGET 0

#include <algorithm>
#include <cmath>
#include <typeinfo>

#if (MCU_TARGET == 0)
#include <iostream>
#include <iomanip>
#endif

namespace grabnum
{

/**
 * A Matlab-alike implementation of a matrix class to simplify and speed up standard
 * operations and make it MCU-friendly.
 *
 * @note Indexing starts at 1 instead of 0 (like in Matlab)!
 */
template <typename T, uint8_t rows, uint8_t cols> class Matrix
{
public:
  using Matrix_t = Matrix<T, rows, cols>;  /**< practical typedef for local use */

  /**
   * Default empty constructor.
   *
   * @todo initialize to zero anyway
   */
  Matrix();
  /**
   * Constructor for empty or identity matrix.
   *
   * @param[in] scalar A scalar value to be duplicated on the diagonal of the matrix. Use
   * 0 for initializing an empty matrix.
   * @see SetZero()
   * @see SetIdentity()
   */
  Matrix(T scalar);
  /**
   * Full constructor.
   * Fills the matrix row-by-row with the elements of @a values.
   *
   * @param[in] values A constant pointer to a constant @a T vector.
   * @param[in] size The number of elements in the vector.
   * @see Fill()
   */
  Matrix(const T* values, const size_t size);
  /**
   * Parametrized constructor from another matrix with the same shape and size.
   * Makes a copy of the given matrix. It also handle automatic casting in case of
   * different types.
   *
   * @param[in] other The copied matrix.
   */
  template <typename T2> Matrix(const Matrix<T2, rows, cols>& other);

  /**
   * Returns numbers of rows.
   *
   * @return A size.
   */
  size_t Rows() const { return rows; }
  /**
   * Returns numbers of rows.
   *
   * @return A size.
   */
  size_t Cols() const { return cols; }
  /**
   * Returns the matrix size, i.e. @a m x @a n.
   *
   * @return A size.
   */
  uint16_t Size() const { return rows * cols; }
  /**
   * Returns the matrix type.
   *
   * @return A constant reference to @c std::type_info.
   * @note This can be useful to compare different matrix types, but can't be used to to
   *declare a variable, for instance.
   */
  const std::type_info& Type() const { return typeid(T); }
  /**
   * Give full access to the matrix data.
   *
   * @return A pointer to the data of the matrix.
   */
  inline T* Data() { return *elements_; }
  /**
   * Give read-only access to the matrix data.
   *
   * @return A pointer to the data of the matrix.
   */
  inline const T* Data() const { return *elements_; }

  /**
   * Conversion operator.
   */
  template <typename NewT> operator Matrix<NewT, rows, cols>();
  /**
   * Assign operator with conversion.
   *
   * @param[in] other The matrix to be copied and type-casted.
   * @return A reference to @c *this.
   */
  template <typename NewT> Matrix_t& operator=(const Matrix<NewT, rows, cols>& other);
  /**
   * Give read-only access to a single entry of the matrix.
   *
   * param[in] row The row index of the desired entry.
   * param[in] col The column index of the desired entry.
   * @return The (@a row , @a col ) entry of the matrix.
   * @note Matrix indexing starts from 1 like in Matlab.
   */
  inline const T& operator()(uint8_t row, uint8_t column) const
  {
    return elements_[row - 1][column - 1];
  }
  /**
   * Give access to a single entry of the matrix.
   *
   * param[in] row The row index of the desired entry.
   * param[in] col The column index of the desired entry.
   * @return The (@a row , @a col ) entry of the matrix.
   * @note Matrix indexing starts from 1 like in Matlab.
   */
  inline T& operator()(uint8_t row, uint8_t column)
  {
    return elements_[row - 1][column - 1];
  }

  /**
   * Operator for element-wise comparison (equal).
   *
   * @param other The matrix to be compared against.
   * @return true if each element of @c *this and @a other are all exactly equal.
   * @warning When using floating point scalar values you probably should rather use a
   *fuzzy
   * comparison such as IsApprox()
   * @see operator!=
   * @see IsApprox()
   */
  bool operator==(const Matrix_t& other) const;
  /**
   * Operator for element-wise comparison (different).
   *
   * @param[in] other The matrix to be compared against.
   * @return true if at least one element of @c *this and @a other are not exactly equal.
   * @warning When using floating point scalar values you probably should rather use a
   *fuzzy
   * comparison such as IsApprox()
   * @see operator==
   * @see IsApprox()
   */
  bool operator!=(const Matrix_t& other) const;
  /**
   * Replaces @c *this by @c *this + the scalar value @a scalar.
   *
   * @param[in] scalar The scalar to be added to each element of the matrix.
   * @return A reference to @c *this.
   */
  Matrix_t& operator+=(const T& scalar);
  /**
   * Replaces @c *this by @c *this + @a other.
   *
   * @param[in] other The matrix to be added to @c *this.
   * @return A reference to @c *this.
   */
  Matrix_t& operator+=(const Matrix_t& other);
  /**
   * Replaces @c *this by @c *this - the scalar value @a scalar.
   *
   * @param[in] scalar The scalar to be subtracted to each element of the matrix.
   * @return A reference to @c *this.
   */
  Matrix_t& operator-=(const T& scalar);
  /**
   * Replaces @c *this by @c *this - @a other.
   *
   * @param[in] other The matrix to be subtracted to @c *this.
   * @return A reference to @c *this.
   */
  Matrix_t& operator-=(const Matrix_t& other);
  /**
   * Replaces @c *this by @c *this * the scalar value @a scalar.
   *
   * @param[in] scalar The factor by which each element of the matrix is multiplied..
   * @return A reference to @c *this.
   */
  Matrix_t& operator*=(const T& scalar);
  /**
   * Replaces @c *this by @c *this divided by the scalar value @a scalar.
   *
   * @param[in] scalar The factor by which each element of the matrix is divided.
   * @return A reference to @c *this.
   */
  Matrix_t& operator/=(const T& scalar);

  /**
   * Replaces a block of @c *this with the elements of @a other.
   *
   * @param[in] start_row The first row in the block.
   * @param[in] start_col The first column in the block.
   * @param[in] other The submatrix to be used to replace the block of  @c *this.
   * @return A reference to @c *this.
   * @note @a block_rows must be <= m - @a start_row, likewise @a block_cols <= n -
   * @a start_col, where @a m x @a n are the dimensions of @c *this. In other words,
   * @a other becomes a submatrix of @c *this.
   * @see SetBlock()
   * @todo example
   */
  template <uint8_t block_rows, uint8_t block_cols>
  Matrix_t& SetBlock(uint8_t start_row, uint8_t start_col,
                     const Matrix<T, block_rows, block_cols>& other);
  /**
   * Sets a column of @c *this with the elements of a 1D matrix.
   *
   * @param[in] col The index of the column to be replaced.
   * @param[in] matrix1d The 1D vertical matrix to be used to replace the column of  @c
   **this.
   * @return A reference to @c *this.
   */
  Matrix_t& SetCol(uint8_t col, const Matrix<T, rows, 1>& matrix1d);
  /**
   * Sets a column of @c *this with the elements of a standard vector.
   *
   * @param[in] col The index of the column to be replaced.
   * @param[in] vect The standard vector to be used to replace the column of  @c *this.
   * @return A reference to @c *this.
   */
  Matrix_t& SetCol(uint8_t col, const T* vect);
  /**
   * Replaces @c *this with a block of @a other.
   *
   * @param[in] start_row The first row in the block.
   * @param[in] start_col The first column in the block.
   * @param[in] other The matrix whose block is used to replace @c *this.
   * @return A reference to @c *this.
   * @note @a m must be <= @a _rows - @a start_row, likewise @a n <= @a _rows -
   * @a start_col, where @a m x @a n are the dimensions of @c *this. In other words,
   * @c *this becomes a submatrix of @a other.
   * @see SetBlock()
   * @todo example
   */
  template <uint8_t _rows, uint8_t _cols>
  Matrix_t& SetFromBlock(uint8_t start_row, uint8_t start_col,
                         const Matrix<T, _rows, _cols>& other);
  /**
   * Set the matrix to an identity matrix.
   *
   * @return A reference to @c *this.
   */
  Matrix_t& SetIdentity();
  /**
   * Sets a row of @c *this with the elements of a 1D matrix.
   *
   * @param[in] row The index of the row to be replaced.
   * @param[in] matrix1d The 1D horizontal matrix to be used to replace the row of  @c
   **this.
   * @return A reference to @c *this.
   */
  Matrix_t& SetRow(uint8_t row, const Matrix<T, 1, cols>& matrix1d);
  /**
   * Sets a row of @c *this with the elements of a standard vector.
   *
   * @param[in] row The index of the row to be replaced.
   * @param[in] vect The standard vector to be used to replace the row of  @c *this.
   * @return A reference to @c *this.
   */
  Matrix_t& SetRow(uint8_t row, const T* vect);
  /**
   * Sets the matrix to an empty matrix.
   *
   * @return A reference to @c *this.
   */
  Matrix_t& SetZero();
  /**
   * Fills the matrix row-by-row with the elements of @a values.
   *
   * @param[in] values A constant pointer to a constant @a T vector.
   * @param[in] size The number of elements in the vector.
   * @return A reference to @c *this.
   */
  Matrix_t& Fill(const T* values, const size_t size);

  /**
   * Returns the transposed matrix.
   *
   * @return The transposed matrix.
   */
  Matrix<T, cols, rows> Transpose() const;
  /**
   * Swaps two different rows of @c *this.
   *
   * @param[in] row1 The index of the first row to be swapped.
   * @param[in] row2 The index of the second row to be swapped.
   * @return A reference to @c *this.
   */
  Matrix_t& SwapRow(uint8_t row1, uint8_t row2);
  /**
   * Swaps two different columns of @c *this.
   *
   * @param[in] col1 The index of the first column to be swapped.
   * @param[in] col2 The index of the second column to be swapped.
   * @return A reference to @c *this.
   */
  Matrix_t& SwapCol(uint8_t col1, uint8_t col2);

  /**
   * Extracts a row from the matrix.
   *
   * @param[in] row The index of the row to be extracted.
   * @return A 1-dimensional matrix (aka an horizontal vector).
   */
  Matrix<T, 1, cols> GetRow(uint8_t row) const;
  /**
   * Extracts a column from the matrix.
   *
   * @param[in] col The index of the column to be extracted.
   * @return A 1-dimensional matrix (aka a vertical vector).
   */
  Matrix<T, rows, 1> GetCol(uint8_t col) const;

  /**
   * Check if the matrix is square, i.e. @a m = @a n.
   *
   * @return true if matrix is square.
   */
  bool IsSquare() const;
  /**
   * Check if the matrix is symmetric, i.e. @a A = @a A^T.
   *
   * @return true if matrix is symmetric.
   */
  bool IsSymmetric() const;
  /**
   * Operator for fuzzy element-wise comparison (equal).
   *
   * @param[in] other The matrix to be compared against.
   * @param[in] tol (optional) The difference below which two single entries are
   *considered equal.
   * @return true if each element of @c *this and @a other are all equal up to a certain
   *tolerance.
   * @see operator!=
   */
  bool IsApprox(const Matrix<T, rows, cols>& other, double tol = 1e-7) const;

private:
  T elements_[rows][cols]; /**< for easy internal access to matrix elements. */
};

///////////////////////////////////////////////////////////////////////////////
/// Matrix utilities
///////////////////////////////////////////////////////////////////////////////

#if (MCU_TARGET == 0)
/**
 * Print function for matrix.
 *
 * @param[in] stream A std output stream.
 * @param[in] matrix A @a m x @a n matrix.
 * @return A reference to the input stream.
 */
template <typename T, uint8_t rows, uint8_t cols>
std::ostream& operator<<(std::ostream& stream, const Matrix<T, rows, cols>& matrix);
#endif

/**
 * Addition between a scalar and a matrix.
 *
 * @param[in] scalar A scalar value.
 * @param[in] matrix A @a m x @a n matrix.
 * @return A @a m x @a n matrix, result of the addition.
 */
template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator+(const T& scalar, const Matrix<T, rows, cols>& matrix);

/**
 * Addition between a matrix and a scalar.
 *
 * @param[in] matrix A @a m x @a n matrix.
 * @param[in] scalar A scalar value.
 * @return A @a m x @a n matrix.
 */
template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator+(const Matrix<T, rows, cols>& matrix, const T& scalar);

/**
 * Addition between two matrices.
 *
 * @param[in] matrix1 A @a m x @a n matrix.
 * @param[in] matrix2 A @a m x @a n matrix.
 * @return A @a m x @a n matrix.
 */
template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator+(const Matrix<T, rows, cols>& matrix1,
                                const Matrix<T, rows, cols>& matrix2);

/**
 * Subtraction between a scalar and a matrix.
 *
 * @param[in] scalar A scalar value.
 * @param[in] matrix A @a m x @a n matrix.
 * @return A @a m x @a n matrix.
 */
template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator-(const T& scalar, const Matrix<T, rows, cols>& matrix);

/**
 * Subtraction between a matrix and a scalar.
 *
 * @param[in] matrix A @a m x @a n matrix.
 * @param[in] scalar A scalar value.
 * @return A @a m x @a n matrix, result of the subtraction.
 */
template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator-(const Matrix<T, rows, cols>& matrix, const T& scalar);

/**
 * Subtraction between two matrices.
 *
 * @param[in] matrix1 A @a m x @a n matrix.
 * @param[in] matrix2 A @a m x @a n matrix.
 * @return A @a m x @a n matrix.
 */
template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator-(const Matrix<T, rows, cols>& matrix1,
                                const Matrix<T, rows, cols>& matrix2);

/**
 * Returns the opposite of a matrix.
 * This is equivalent to multiplication by -1.
 *
 * @param[in] matrix A @a m x @a n matrix.
 * @return A @a m x @a n matrix.
 */
template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator-(const Matrix<T, rows, cols>& matrix);

/**
 * Row-column matrix multiplication.
 *
 * @param[in] matrix1 A @a m x @a n matrix.
 * @param[in] matrix2 A @a n x @a p matrix.
 * @return A @a m x @a p matrix.
 */
template <typename T, uint8_t rows1, uint8_t dim_common, uint8_t cols2>
Matrix<T, rows1, cols2> operator*(const Matrix<T, rows1, dim_common>& matrix1,
                                  const Matrix<T, dim_common, cols2>& matrix2);

/**
 * Outer product operation.
 *
 * @param[in] vvect A m-dimensional vertical vector.
 * @param[in] hvect A n-dimensional horizontal vector.
 * @return A @a m x @a n matrix.
 */
template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator*(const Matrix<T, rows, 1>& vvect,
                                const Matrix<T, 1, cols>& hvect);

/**
 * Scalar product operation.
 *
 * @param[in] scalar A scalar value.
 * @param[in] matrix A @a m x @a n matrix.
 * @return A @a m x @a n matrix.
 */
template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator*(const T& scalar, const Matrix<T, rows, cols>& matrix);

/**
 * Scalar product operation.
 *
 * @param[in] matrix A @a m x @a n matrix.
 * @param[in] scalar A scalar value.
 * @return A @a m x @a n matrix.
 */
template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator*(const Matrix<T, rows, cols>& matrix, const T& scalar);

/**
 * Element-wise vector multiplication.
 *
 * @param[in] vvect1 A m-dimensional vertical vector.
 * @param[in] vvect2 A m-dimensional vertical vector.
 * @return A m-dimensional vertical vector.
 */
template <uint8_t rows, typename T>
Matrix<T, rows, 1> operator*(const Matrix<T, rows, 1>& vvect1,
                             const Matrix<T, rows, 1>& vvect2);

/**
 * Matrix division by scalar operation.
 *
 * @param[in] matrix A @a m x @a n matrix.
 * @param[in] scalar A scalar value.
 * @return A @a m x @a n matrix.
 */
template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator/(const Matrix<T, rows, cols>& matrix, const T& scalar);

/**
 * Matrix horizontal concatenation.
 *
 * @param[in] matrix_lx A @a m x @a n matrix.
 * @param[in] matrix_rx A @a m x @a p matrix.
 * @return A @a m x @a (n+p) matrix.
 */
template <typename T, uint8_t rows, uint8_t cols1, uint8_t cols2>
Matrix<T, rows, cols1 + cols2> HorzCat(const Matrix<T, rows, cols1>& matrix_lx,
                                       const Matrix<T, rows, cols2>& matrix_rx);

/**
 * Matrix vertical concatenation.
 *
 * @param[in] matrix_up A @a m x @a n matrix.
 * @param[in] matrix_down A @a p x @a n matrix.
 * @return A @a (m+p) x @a n matrix.
 */
template <typename T, uint8_t rows1, uint8_t rows2, uint8_t cols>
Matrix<T, rows1 + rows2, cols> VertCat(const Matrix<T, rows1, cols>& matrix_up,
                                       const Matrix<T, rows2, cols>& matrix_down);

/**
 * Vector dot-product operation.
 *
 * @param[in] vvect1 A m-dimensional vertical vector.
 * @param[in] vvect2 A m-dimensional vertical vector.
 * @return A m-dimensional vertical vector.
 */
template <typename T, uint8_t dim>
T Dot(Matrix<T, dim, 1>& vvect1, Matrix<T, dim, 1>& vvect2);

/**
 * Vector dot-product operation.
 *
 * @param[in] hvect A m-dimensional horizontal vector.
 * @param[in] vvect A m-dimensional vertical vector.
 * @return A m-dimensional vertical vector.
 */
template <typename T, uint8_t dim>
T Dot(const Matrix<T, 1, dim>& hvect, const Matrix<T, dim, 1>& vvect);

/**
 * Vector L2-norm (i.e. Euclidean norm).
 *
 * @param[in] vvect A m-dimensional vertical vector.
 * @return A scalar value.
 */
template <typename T, uint8_t dim> T Norm(const Matrix<T, dim, 1>& vvect);

/**
 * Vector L2-norm (i.e. Euclidean norm).
 *
 * @param[in] hvect A m-dimensional horizontal vector.
 * @return A scalar value.
 */
template <typename T, uint8_t dim> T Norm(const Matrix<T, 1, dim>& hvect);

/**
 * Vector cross-product operation.
 *
 * @param[in] vvect3d1 A 3-dimensional vertical vector.
 * @param[in] vvect2 A 3-dimensional vertical vector.
 * @return A 3x3 matrix.
 */
template <typename T>
Matrix<T, 3, 1> Cross(const Matrix<T, 3, 1>& vvect3d1, const Matrix<T, 3, 1>& vvect3d2);

/**
 * Computes the anti-symmetric matrix of a 3D vector.
 *
 * @param[in] vvect3d A 3-dimensional vertical vector.
 * @return A 3x3 matrix.
 */
template <typename T> Matrix<T, 3, 3> Anti(const Matrix<T, 3, 1>& vvect3d);

} //  end namespace grabnum

#include "matrix.cpp"

#endif /* GRABCOMMON_LIBNUMERIC_MATRIX_H_ */
