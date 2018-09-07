/**
 * @file matrix.h
 * @author Edoardo Idà, Simone Comari
 * @date 07 Sep 2018
 * @brief File containing matrix class and utilities to be included in the GRAB numeric
 * library.
 *
 * In this file a simple implementation of a class object is provided, with its basic
 * functionalities.
 * Below the class there are some external independent functions which provide extra
 * utilities related to matrix operations. All elements of this file are templated to be
 * compatible with all different types and leave the user more flexibility.
 */

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

#include "common.h"

#if (MCU_TARGET == 0)
#include <iostream>
#include <iomanip>
#endif

#ifndef SQUARE
#define SQUARE(x) (x * x) /**< returns the square of an element. */
#endif

/**
 * Namespace for GRAB numeric library.
 */
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
  using Matrix_t = Matrix<T, rows, cols>; /**< practical typedef for local use */

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
  Matrix(const T* values, const uint16_t size);
  /**
   * Full constructor.
   * Fills the matrix row-by-row with the elements of @a values.
   *
   * @param[in] values A standard @a T vector.
   * @see Fill()
   */
  Matrix(const std::vector<T>& values);
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
  uint8_t Rows() const { return rows; }
  /**
   * Returns numbers of rows.
   *
   * @return A size.
   */
  uint8_t Cols() const { return cols; }
  /**
   * Returns the matrix size, i.e. @a m x @a n.
   *
   * @return A size.
   */
  uint16_t Size() const { return rows * cols; }
  /**
   * Returns the maximum value inside the matrix.
   *
   * @return The maximum value.
   */
  inline T Max() const { return this(MaxIdx()); }
  /**
   * Returns the minimum value inside the matrix.
   *
   * @return The minimum value.
   */
  inline T Min() const { return this(MinIdx()); }
  /**
   * Returns the linear index of the maximum value inside the matrix.
   *
   * @return The linear index of the maximum value.
   */
  uint16_t MaxIdx() const;
  /**
   * Returns the linear index of the minimum value inside the matrix.
   *
   * @return The linear index of the minimum value.
   */
  uint16_t MinIdx() const;
  /**
   * Returns the linear index corresponding to a standard double index.
   *
   * @return The linear index corresponding to a standard double index.
   */
  inline uint16_t LinIdx(const uint8_t row, const uint8_t col) const
  { return (row - 1) * cols + col; }
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
  template <typename NewT>
  Matrix<T, rows, cols>& operator=(const Matrix<NewT, rows, cols>& other);
  /**
   * Give read-only access to a single entry of the matrix.
   *
   * param[in] row The row index of the desired entry.
   * param[in] col The column index of the desired entry.
   * @return The (@a row , @a col ) entry of the matrix.
   * @note Matrix indexing starts from 1 like in Matlab.
   */
  inline const T& operator()(const uint8_t row, const uint8_t column) const
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
  inline T& operator()(const uint8_t row, const uint8_t column)
  {
    return elements_[row - 1][column - 1];
  }
  /**
   * Give read-only access to a single entry of the unraveled matrix.
   * This is particularly useful when accessing elements of a vector (aka 1D matrix).
   *
   * param[in] lin_index The linear index of the desired entry.
   * @return The i-th entry of the matrix.
   * @note Matrix indexing starts from 1 like in Matlab and read row-by-row,
   *top-to-bottom.
   */
  inline const T& operator()(const uint8_t lin_index) const
  {
    uint8_t row = (lin_index - 1) / cols;
    uint8_t column = (lin_index - 1) % cols;
    return elements_[row][column];
  }
  /**
   * Give access to a single entry of the unraveled matrix.
   * This is particularly useful when accessing elements of a vector (aka 1D matrix).
   *
   * param[in] lin_index The linear index of the desired entry.
   * @return The i-th entry of the matrix.
   * @note Matrix indexing starts from 1 like in Matlab and read row-by-row,
   *top-to-bottom.
   */
  inline T& operator()(const uint8_t lin_index)
  {
    uint8_t row = (lin_index - 1) / cols;
    uint8_t column = (lin_index - 1) % cols;
    return elements_[row][column];
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
   * @see SetFromBlock()
   * @todo example
   */
  template <uint8_t block_rows, uint8_t block_cols>
  Matrix_t& SetBlock(const uint8_t start_row, const uint8_t start_col,
                     const Matrix<T, block_rows, block_cols>& other);
  /**
   * Sets a column of @c *this with the elements of a 1D matrix.
   *
   * @param[in] col The index of the column to be replaced.
   * @param[in] matrix1d The 1D vertical matrix to be used to replace the column of  @c
   **this.
   * @return A reference to @c *this.
   */
  Matrix_t& SetCol(const uint8_t col, const Matrix<T, rows, 1>& matrix1d);
  /**
   * Sets a column of @c *this with the elements of a standard vector.
   *
   * @param[in] col The index of the column to be replaced.
   * @param[in] vect The standard vector to be used to replace the column of  @c *this.
   * @param[in] size The length of the column vector. It needs to be the same as the
   *number of rows in the original matrix.
   * @return A reference to @c *this.
   */
  Matrix_t& SetCol(const uint8_t col, const T* vect, const uint8_t size);
  /**
   * Sets a column of @c *this with the elements of a standard vector.
   *
   * @param[in] col The index of the column to be replaced.
   * @param[in] vect The standard vector to be used to replace the column of  @c *this.
   * @return A reference to @c *this.
   */
  Matrix_t& SetCol(const uint8_t col, const std::vector<T>& vect);
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
  Matrix<T, rows, cols>& SetFromBlock(const uint8_t start_row, const uint8_t start_col,
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
  Matrix_t& SetRow(const uint8_t row, const Matrix<T, 1, cols>& matrix1d);
  /**
   * Sets a row of @c *this with the elements of a standard vector.
   *
   * @param[in] row The index of the row to be replaced.
   * @param[in] vect A pointer to the vector to be used to replace the row of  @c *this.
   * @param[in] size The length of the row vector. It needs to be the same as the number
   *of rows in the original matrix.
   * @return A reference to @c *this.
   */
  Matrix_t& SetRow(const uint8_t row, const T* vect, const uint8_t size);
  /**
   * Sets a row of @c *this with the elements of a standard vector.
   *
   * @param[in] row The index of the row to be replaced.
   * @param[in] vect The standard vector to be used to replace the row of  @c *this.
   * @return A reference to @c *this.
   */
  Matrix_t& SetRow(const uint8_t row, const std::vector<T>& vect);
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
  Matrix_t& Fill(const T* values, const uint16_t size);
  /**
   * Fills the matrix row-by-row with the elements of @a values.
   *
   * @param[in] values A reference to a standard @a T vector.
   * @return A reference to @c *this.
   */
  Matrix_t& Fill(const std::vector<T>& values);

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
  Matrix_t& SwapRow(const uint8_t row1, const uint8_t row2);
  /**
   * Swaps two different columns of @c *this.
   *
   * @param[in] col1 The index of the first column to be swapped.
   * @param[in] col2 The index of the second column to be swapped.
   * @return A reference to @c *this.
   */
  Matrix_t& SwapCol(const uint8_t col1, const uint8_t col2);

  /**
   * Extracts a row from the matrix.
   *
   * @param[in] row The index of the row to be extracted.
   * @return A 1-dimensional matrix (aka an horizontal vector).
   */
  Matrix<T, 1, cols> GetRow(const uint8_t row) const;
  /**
   * Extracts a column from the matrix.
   *
   * @param[in] col The index of the column to be extracted.
   * @return A 1-dimensional matrix (aka a vertical vector).
   */
  Matrix<T, rows, 1> GetCol(const uint8_t col) const;
  /**
   * Extracts a block from the matrix.
   *
   * The number of rows and columns of the block are specified in the template arguments.
   * @param[in] start_row The starting row of the block.
   * @param[in] start_col The starting column of the block.
   * @return A block of the original matrix.
   */
  template<uint8_t blk_rows, uint8_t blk_cols>
  Matrix<T, blk_rows, blk_cols> GetBlock(const uint8_t start_row, const uint8_t start_col) const;

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
  bool IsApprox(const Matrix<T, rows, cols>& other, const double tol = EPSILON) const;

private:
  T elements_[rows][cols]; /**< for easy internal access to matrix elements. */
};

///////////////////////////////////////////////////////////////////////////////
/// Common typedef
///////////////////////////////////////////////////////////////////////////////

template <typename T> using Vector2 = Matrix<T, 2, 1>; /**< generic 2D vector */
using Vector2i = Vector2<int>;                         /**< 2D vector of int */
using Vector2l = Vector2<long>;                        /**< 2D vector of long */
using Vector2f = Vector2<float>;                       /**< 2D vector of float */
using Vector2d = Vector2<double>;                      /**< 2D vector of double */

template <typename T> using Vector3 = Matrix<T, 3, 1>; /**< generic 3D vector */
using Vector3i = Vector3<int>;                         /**< 3D vector of int */
using Vector3l = Vector3<long>;                        /**< 3D vector of long */
using Vector3f = Vector3<float>;                       /**< 3D vector of float */
using Vector3d = Vector3<double>;                      /**< 3D vector of double */

template <typename T> using Matrix2 = Matrix<T, 2, 2>; /**< generic 2x2 matrix */
using Matrix2i = Matrix2<int>;                         /**< 2x2 matrix of int */
using Matrix2l = Matrix2<long>;                        /**< 2x2 matrix of long */
using Matrix2f = Matrix2<float>;                       /**< 2x2 matrix of float */
using Matrix2d = Matrix2<double>;                      /**< 2x2 matrix of double */

template <typename T> using Matrix3 = Matrix<T, 3, 3>; /**< generic 3x3 matrix */
using Matrix3i = Matrix3<int>;                         /**< 3x3 matrix of int */
using Matrix3l = Matrix3<long>;                        /**< 3x3 matrix of long */
using Matrix3f = Matrix3<float>;                       /**< 3x3 matrix of float */
using Matrix3d = Matrix3<double>;                      /**< 3x3 matrix of double */

template <uint8_t rows, uint8_t cols>
using MatrixXi = Matrix<int, rows, cols>; /**< generic mxn matrix of int */
template <uint8_t rows, uint8_t cols>
using MatrixXl = Matrix<long, rows, cols>; /**< generic mxn matrix of long */
template <uint8_t rows, uint8_t cols>
using MatrixXf = Matrix<float, rows, cols>; /**< generic mxn matrix of float */
template <uint8_t rows, uint8_t cols>
using MatrixXd = Matrix<double, rows, cols>; /**< generic mxn matrix of double */

template <typename T, uint8_t dim>
using VectorX = Matrix<T, dim, 1>;                           /**< generic vector */
template <uint8_t dim> using VectorXi = Matrix<int, dim, 1>; /**< generic vector of int */
template <uint8_t dim>
using VectorXl = Matrix<long, dim, 1>; /**< generic vector of long */
template <uint8_t dim>
using VectorXf = Matrix<float, dim, 1>; /**< generic vector of float */
template <uint8_t dim>
using VectorXd = Matrix<double, dim, 1>; /**< generic vector of double */

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
Matrix<T, rows, cols> operator*(const VectorX<T, rows>& vvect,
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
template <typename T, uint8_t rows>
VectorX<T, rows> operator*(const VectorX<T, rows>& vvect1,
                           const VectorX<T, rows>& vvect2);

/**
 * Element-wise vector multiplication.
 *
 * @param[in] hvect1 A m-dimensional horizontal vector.
 * @param[in] hvect2 A m-dimensional horizontal vector.
 * @return A m-dimensional horizontal vector.
 */
template <typename T, uint8_t cols>
Matrix<T, 1, cols> operator*(const Matrix<T, 1, cols>& hvect1,
                             const Matrix<T, 1, cols>& hvect2);

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
 * @return A scalar value.
 */
template <typename T, uint8_t dim>
T Dot(const VectorX<T, dim>& vvect1, const VectorX<T, dim>& vvect2);

/**
 * Vector dot-product operation.
 *
 * @param[in] hvect A m-dimensional horizontal vector.
 * @param[in] vvect A m-dimensional vertical vector.
 * @return A scalar value.
 */
template <typename T, uint8_t dim>
T Dot(const Matrix<T, 1, dim>& hvect, const VectorX<T, dim>& vvect);

/**
 * Vector L2-norm (i.e. Euclidean norm).
 *
 * @param[in] vvect A m-dimensional vertical vector.
 * @return A scalar value.
 */
template <typename T, uint8_t dim> double Norm(const VectorX<T, dim>& vvect);

/**
 * Vector L2-norm (i.e. Euclidean norm).
 *
 * @param[in] hvect A m-dimensional horizontal vector.
 * @return A scalar value.
 */
template <typename T, uint8_t dim> double Norm(const Matrix<T, 1, dim>& hvect);

/**
 * Vector cross-product operation.
 *
 * @param[in] vvect3d1 A 3-dimensional vertical vector.
 * @param[in] vvect3d2 A 3-dimensional vertical vector.
 * @return A 3x3 matrix.
 */
template <typename T>
Vector3<T> Cross(const Vector3<T>& vvect3d1, const Vector3<T>& vvect3d2);

/**
 * Computes the skew-symmetric matrix of a 3D vector.
 *
 * @param[in] vvect3d A 3-dimensional vertical vector.
 * @return A 3x3 matrix.
 */
template <typename T> Matrix3<T> Skew(const Vector3<T>& vvect3d);

} //  end namespace grabnum

// This is a trick to define templated functions in a source file.
#include "matrix.cpp"

#endif /* GRABCOMMON_LIBNUMERIC_MATRIX_H_ */
