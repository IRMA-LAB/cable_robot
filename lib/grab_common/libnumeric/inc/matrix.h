/*
 * EasyMatrix.h
 *
 *  Created on: 12 Jun 2018
 *      Author: Edoardo Idà
 */

#ifndef GRABCOMMON_LIBNUMERIC_MATRIX_H_
#define GRABCOMMON_LIBNUMERIC_MATRIX_H_

#include <algorithm>
#include <cmath>

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

  using Matrix_t = Matrix<T, rows, cols>;

  /**
   * Default empty constructor.
   *
   * @todo initialize to zero anyway
   */
  Matrix();
  /**
   * Constructor for empty or identity matrix.
   *
   * @param[in] scalar A scalar value to be duplicated on the diagonal of the matrix. Use 0 for
   * initializing an empty matrix.
   */
  Matrix(T scalar);
  /**
   * Full constructor.
   * Fills the matrix row-by-row with the elements of @a values.
   *
   * @param[in] values A constant pointer to a constant @a T vector.
   */
  Matrix(const T *const values);

  /**
   * Give full access to the matrix data.
   *
   * @return A pointer to the data of the matrix.
   */
  inline T* data() { return *elements; }
  /**
   * Give read-only access to the matrix data.
   *
   * @return A pointer to the data of the matrix.
   */
  inline const T* data() const { return *elements; }

  /**
   * Give read-only access to a single entry of the matrix.
   *
   * param[in] row The row index of the desired entry.
   * param[in] col The column index of the desired entry.
   * @return The (@a row , @a col ) entry of the matrix.
   */
  inline const T& operator()(int row, int column) const
  {
    return elements[row - 1][column - 1];
  }
  /**
   * Give access to a single entry of the matrix.
   *
   * param[in] row The row index of the desired entry.
   * param[in] col The column index of the desired entry.
   * @return The (@a row , @a col ) entry of the matrix.
   */
  inline T& operator()(int row, int column) { return elements[row - 1][column - 1]; }

  /**
   * Operator for element-wise comparison (equal).
   *
   * @param other The matrix to be compared against.
   * @return true if each element of @c *this and @a other are all exactly equal.
   * @warning When using floating point scalar values you probably should rather use a fuzzy
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
   * @warning When using floating point scalar values you probably should rather use a fuzzy
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
  template <uint8_t block_rows, uint8_t block_cols> Matrix_t& SetBlock(
      uint8_t start_row, uint8_t start_col,  const Matrix<T, block_rows, block_cols>& other);
  /**
   * Sets a column of @c *this with the elements of a 1D matrix.
   *
   * @param[in] col The index of the column to be replaced.
   * @param[in] matrix1d The 1D vertical matrix to be used to replace the column of  @c *this.
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
  template <uint8_t _rows, uint8_t _cols> Matrix_t& SetFromBlock(
      uint8_t start_row, uint8_t start_col, const Matrix<T, _rows, _cols>& other);
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
   * @param[in] matrix1d The 1D horizontal matrix to be used to replace the row of  @c *this.
   * @return A reference to @c *this.
   */
  Matrix_t& SetRow(uint8_t row, const Matrix<T, 1, cols>& matrix1d);
  /**
   * Sets a row of @c *this with the elements of a standard vector.
   *
   * @param[in] row The index of the row to be replaced.
   * @param[in] matrix1d The standard vector to be used to replace the row of  @c *this.
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
   * @return A reference to @c *this.
   */
  Matrix_t& Fill(const T* values);

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
  Matrix<T, 1, cols> GetRow(uint8_t row);
  /**
   * Extracts a column from the matrix.
   *
   * @param[in] col The index of the column to be extracted.
   * @return A 1-dimensional matrix (aka a vertical vector).
   */
  Matrix<T, rows, 1> GetCol(uint8_t col);

private:

  T elements[rows][cols];  /**< for easy internal access to matrix elements. */
};

/////////////////////////////////////////////////////////////////// Matrix utilities

/*#include <iostream>
#include <iomanip>

template <int Row, int Col, typename T>
std::ostream& operator<<(std::ostream &stream, const EasyMatrix<Row, Col, T> &matrix)
{

  for (int row = 1; row<=Row; row++) {
      if (row==1) stream << "[";
      else stream << " ";
      for (int col = 1; col <= Col; col++) {
          stream << std::setw(15) << std::setprecision(7) << matrix(row,col);
          if (row==Row && col==Col)
              stream << "         ]\n";
      }
      stream << "\n";
  }
  stream << "\n";
  return stream;
}*/

template <int rows, int cols, typename T>
Matrix<T, rows, cols> operator+(const T& add, const Matrix<T, rows, cols>& m1)
{
  Matrix<T, rows, cols> result;
  for (int row = 0; row < rows; row++)
    for (int col = 0; col < cols; col++)
      result.elements[row][col] = m1.elements[row][col] + add;
  return result;
}

template <int rows, int cols, typename T>
Matrix<T, rows, cols> operator+(const Matrix<T, rows, cols>& m1, const T& add)
{
  Matrix<T, rows, cols> result;
  for (int row = 0; row < rows; row++)
    for (int col = 0; col < cols; col++)
      result.elements[row][col] = m1.elements[row][col] + add;
  return result;
}

template <int rows, int cols, typename T>
Matrix<T, rows, cols> operator+(const Matrix<T, rows, cols>& m1,
                                const Matrix<T, rows, cols>& m2)
{
  Matrix<T, rows, cols> result;
  for (int row = 0; row < rows; row++)
    for (int col = 0; col < cols; col++)
      result.elements[row][col] = m1.elements[row][col] + m2.elements[row][col];
  return result;
}

template <int rows, int cols, typename T>
Matrix<T, rows, cols> operator-(const T& subtract, const Matrix<T, rows, cols>& m1)
{
  Matrix<T, rows, cols> result;
  for (int row = 0; row < rows; row++)
    for (int col = 0; col < cols; col++)
      result.elements[row][col] = subtract - m1.elements[row][col];
  return result;
}

template <int rows, int cols, typename T>
Matrix<T, rows, cols> operator-(const Matrix<T, rows, cols>& m1, const T& subtract)
{
  Matrix<T, rows, cols> result;
  for (int row = 0; row < rows; row++)
    for (int col = 0; col < cols; col++)
      result.elements[row][col] = m1.elements[row][col] - subtract;
  return result;
}

template <int rows, int cols, typename T>
Matrix<T, rows, cols> operator-(const Matrix<T, rows, cols>& m1,
                                const Matrix<T, rows, cols>& m2)
{
  Matrix<T, rows, cols> result;
  for (int row = 0; row < rows; row++)
    for (int col = 0; col < cols; col++)
      result.elements[row][col] = m1.elements[row][col] - m2.elements[row][col];
  return result;
}

template <int rows, int cols, typename T>
Matrix<T, rows, cols> operator-(const Matrix<T, rows, cols>& m1)
{
  Matrix<T, rows, cols> result;
  for (int row = 0; row < rows; row++)
    for (int col = 0; col < cols; col++)
      result.elements[row][col] = -m1.elements[row][col];
  return result;
}

template <int Row1, int Com, int Col2, typename T>
Matrix<Row1, Col2, T> operator*(const Matrix<Row1, Com, T>& m1,
                                const Matrix<Com, Col2, T>& m2)
{
  Matrix<Row1, Col2, T> result;
  for (int row = 0; row < Row1; row++)
  {
    for (int col = 0; col < Col2; col++)
    {
      T sum = 0;
      for (int j = 0; j < Com; j++)
        sum += m1.elements[row][j] * m2.elements[j][col];
      result.elements[row][col] = sum;
    }
  }
  return result;
}

template <int rows, int cols, typename T>
Matrix<T, rows, cols> operator*(const Matrix<rows, 1, T>& m1,
                                const Matrix<1, cols, T>& m2)
{
  Matrix<T, rows, cols> result;
  for (int row = 0; row < rows; row++)
  {
    for (int col = 0; col < cols; col++)
    {
      result.elements[row][col] = m1.elements[row][0] * m2.elements[0][col];
    }
  }
  return result;
}

template <int rows, int cols, typename T>
Matrix<T, rows, cols> operator*(const T& factor, const Matrix<T, rows, cols>& m1)
{
  Matrix<T, rows, cols> result;
  for (int row = 0; row < rows; row++)
    for (int col = 0; col < cols; col++)
      result.elements[row][col] = m1.elements[row][col] * factor;
  return result;
}

template <int rows, int cols, typename T>
Matrix<T, rows, cols> operator*(const Matrix<T, rows, cols>& m1, const T& factor)
{
  Matrix<T, rows, cols> result;
  for (int row = 0; row < rows; row++)
    for (int col = 0; col < cols; col++)
      result.elements[row][col] = m1.elements[row][col] * factor;
  return result;
}

template <int rows, typename T>
Matrix<rows, 1, T> operator*(const Matrix<rows, 1, T>& m1, const Matrix<rows, 1, T>& m2)
{
  Matrix<rows, 1, T> result;
  for (int row = 0; row < rows; row++)
    for (int col = 0; col < 1; col++)
      result.elements[row][col] = m1.elements[row][col] * m2.elements[row][col];
  return result;
}

template <int rows, int cols, typename T>
Matrix<T, rows, cols> operator/(const Matrix<T, rows, cols>& m1, const T& divisor)
{
  Matrix<T, rows, cols> result;
  for (int row = 0; row < rows; row++)
    for (int col = 0; col < cols; col++)
      result.elements[row][col] = m1.elements[row][col] / divisor;
  return result;
}

template <int rows, int Col1, int Col2, typename T>
Matrix<rows, Col1 + Col2, T> HorzCat(const Matrix<rows, Col1, T>& m1,
                                     const Matrix<rows, Col2, T>& m2)
{
  Matrix<rows, Col1 + Col2, T> result;

  for (int row = 0; row < rows; row++)
  {
    for (int col = 0; col < Col1; col++)
      result.elements[row][col] = m1.elements[row][col];
    for (int col = 0; col < Col2; col++)
      result.elements[row][col + Col1] = m2.elements[row][col];
  }
  return result;
}

template <int Row1, int Row2, int cols, typename T>
Matrix<Row1 + Row2, cols, T> VertCat(const Matrix<Row1, cols, T>& m1,
                                     const Matrix<Row2, cols, T>& m2)
{
  Matrix<Row1 + Row2, cols, T> result;
  for (int col = 0; col < cols; col++)
  {
    for (int row = 0; row < Row1; row++)
      result.elements[row][col] = m1.elements[row][col];
    for (int row = 0; row < Row2; row++)
      result.elements[row + Row1][col] = m2.elements[row][col];
  }
  return result;
}

template <int Dim, typename T> T Dot(Matrix<Dim, 1, T>& m1, Matrix<Dim, 1, T>& m2)
{
  T result = 0;

  for (int i = 1; i <= Dim; i++)
  {
    result += m1(i, 1) * m2(i, 1);
  }

  return result;
}

template <int Dim, typename T>
T Dot(const Matrix<1, Dim, T>& m1, const Matrix<Dim, 1, T>& m2)
{
  T result = 0;

  for (int i = 1; i <= Dim; i++)
  {
    result += m1(1, i) * m2(i, 1);
  }

  return result;
}

template <int Dim, typename T> T Norm(const Matrix<Dim, 1, T>& mat)
{
  T result = 0;
  for (int i = 1; i <= Dim; i++)
    result += mat(i, 1) * mat(i, 1);
  return sqrt(result);
}

template <int Dim, typename T> T Norm(const Matrix<1, Dim, T>& mat)
{
  T result = 0;
  for (int i = 1; i <= Dim; i++)
    result += mat(1, i) * mat(1, i);
  return sqrt(result);
}

template <typename T>
Matrix<3, 1, T> Cross(const Matrix<3, 1, T>& m1, const Matrix<3, 1, T>& m2)
{

  return Anti(m1) * m2;
}

template <typename T> Matrix<3, 3, T> Anti(const Matrix<3, 1, T>& m1)
{
  Matrix<3, 3, T> result;
  result(1, 1) = 0;
  result(2, 2) = 0;
  result(3, 3) = 0;
  result(1, 2) = -m1(3, 1);
  result(2, 1) = m1(3, 1);
  result(1, 3) = m1(2, 1);
  result(3, 1) = -m1(2, 1);
  result(2, 3) = -m1(1, 1);
  result(3, 2) = m1(1, 1);
  return result;
}

} //  end namespace grabnum

#endif /* GRABCOMMON_LIBNUMERIC_MATRIX_H_ */
