#include <assert.h>
#include "matrix.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof((x)) / sizeof((x)[0]))
#endif

namespace grabnum
{

///////////////////////////////////////////////////////////////////////////////
/// Constructors
///////////////////////////////////////////////////////////////////////////////

template <typename T, uint8_t rows, uint8_t cols> Matrix<T, rows, cols>::Matrix()
{
  SetZero();
}

template <typename T, uint8_t rows, uint8_t cols> Matrix<T, rows, cols>::Matrix(T scalar)
{
  if (scalar == 0)
    SetZero();
  else
    *this = SetIdentity() * scalar;
}

  template <typename T, uint8_t rows, uint8_t cols>
  Matrix<T, rows, cols>::Matrix(const T* values, const size_t size)
  {
    Fill(values, size);
  }

  template <typename T, uint8_t rows, uint8_t cols>
  template <typename T2>
  Matrix<T, rows, cols>::Matrix(const Matrix<T2, rows, cols>& other)
  {
  }

///////////////////////////////////////////////////////////////////////////////
/// Operator Overloadings
///////////////////////////////////////////////////////////////////////////////

template <typename T, uint8_t rows, uint8_t cols>
bool Matrix<T, rows, cols>::operator==(const Matrix<T, rows, cols>& other) const
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
    {
      if (elements_[row][col] != other(row + 1, col + 1))
        return false;
    }
  return true;
}

template <typename T, uint8_t rows, uint8_t cols>
bool Matrix<T, rows, cols>::operator!=(const Matrix<T, rows, cols>& other) const
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
    {
      if (elements_[col][row] != other.elements_[col][row])
        return true;
    }
  return false;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::operator+=(const T& scalar)
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements_[row][col] += scalar;
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::
operator+=(const Matrix<T, rows, cols>& other)
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements_[row][col] += other(row + 1, col + 1);
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::operator-=(const T& scalar)
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements_[row][col] -= scalar;
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::
operator-=(const Matrix<T, rows, cols>& other)
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements_[row][col] -= other(row + 1, col + 1);
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::operator*=(const T& scalar)
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements_[row][col] *= scalar;
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::operator/=(const T& scalar)
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements_[row][col] /= scalar;
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
/// Setters
///////////////////////////////////////////////////////////////////////////////

template <typename T, uint8_t rows, uint8_t cols>
template <uint8_t block_rows, uint8_t block_cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetBlock(
  uint8_t start_row, uint8_t start_col, const Matrix<T, block_rows, block_cols>& other)
{
  for (uint8_t row = start_row; row < start_row + block_rows; ++row)
    for (uint8_t col = start_col; col < start_col + block_cols; ++col)
      elements_[row - 1][col - 1] = other.elements_[row - start_row][col - start_col];
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetCol(uint8_t cl,
                                                     const Matrix<T, rows, 1>& matrix1d)
{
  for (uint8_t i = 0; i < rows; i++)
  {
    elements_[i][cl - 1] = matrix1d.elements_[i][0];
  }
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetCol(uint8_t cl, const T* vect)
{
  for (uint8_t i = 0; i < rows; i++)
  {
    elements_[i][cl - 1] = vect[i];
  }
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
template <uint8_t _rows, uint8_t _cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetFromBlock(
  uint8_t start_row, uint8_t start_col, const Matrix<T, _rows, _cols>& other)
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements_[row][col] = other.elements_[row + start_row - 1][col + start_col - 1];
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetIdentity()
{
#if (MCU_TARGET == 0)
  if (!IsSquare())
  {
    std::cerr << "WARNING: Matrix is not square! Pseudo-identity matrix is generated."
              << std::endl;
  }
#endif
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
    {
      if (row == col)
        elements_[row][col] = 1;
      else
        elements_[row][col] = 0;
    }
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetRow(uint8_t rw,
                                                     const Matrix<T, 1, cols>& matrix1d)
{
  for (uint8_t i = 0; i < cols; i++)
  {
    elements_[rw - 1][i] = matrix1d.elements_[0][i];
  }
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetRow(uint8_t rw, const T* vect)
{
  for (uint8_t i = 0; i < cols; i++)
  {
    elements_[rw - 1][i] = vect[i];
  }
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetZero()
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements_[row][col] = 0;
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::Fill(const T* values, const size_t size)
{
  assert(size == Size());

  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements_[row][col] = values[row * cols + col];
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
/// Matrix manipulation
///////////////////////////////////////////////////////////////////////////////

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, cols, rows> Matrix<T, rows, cols>::Transpose() const
{
  Matrix<T, cols, rows> transpose;
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      transpose(col + 1, row + 1) = elements_[row][col];
  return transpose;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SwapRow(uint8_t row1, uint8_t row2)
{
  T temp;
  for (uint8_t i = 0; i < cols; i++)
  {
    temp = elements_[row1 - 1][i];
    elements_[row1 - 1][i] = elements_[row2 - 1][i];
    elements_[row1 - 1][i] = temp;
  }
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SwapCol(uint8_t col1, uint8_t col2)
{
  T temp;
  for (uint8_t i = 0; i < rows; i++)
  {
    temp = elements_[i][col1 - 1];
    elements_[i][col1 - 1] = elements_[i][col2 - 1];
    elements_[i][col2 - 1] = temp;
  }
  return *this;
}

///////////////////////////////////////////////////////////////////////////////
/// Getters
///////////////////////////////////////////////////////////////////////////////

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, 1, cols> Matrix<T, rows, cols>::GetRow(uint8_t row)
{
  Matrix<T, 1, cols> row_vect;
  for (uint8_t col = 0; col < cols; ++col)
    row_vect(1, col + 1) = elements_[row - 1][col];
  return row_vect;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, 1> Matrix<T, rows, cols>::GetCol(uint8_t col)
{
  Matrix<T, rows, 1> col_vect;
  for (uint8_t row = 0; row < rows; ++row)
    col_vect(row + 1, 1) = elements_[row][col - 1];
  return col_vect;
}

///////////////////////////////////////////////////////////////////////////////
/// Check functions
///////////////////////////////////////////////////////////////////////////////

template <typename T, uint8_t rows, uint8_t cols>
bool Matrix<T, rows, cols>::IsSquare()
{
  return rows == cols;
}

template <typename T, uint8_t rows, uint8_t cols>
bool Matrix<T, rows, cols>::IsSymmetric()
{
  if (!IsSquare())
    return false;
  return *this == Transpose();
}

///////////////////////////////////////////////////////////////////////////////
/// Matrix external utilities
///////////////////////////////////////////////////////////////////////////////

#if (MCU_TARGET == 0)
template <typename T, uint8_t rows, uint8_t cols>
std::ostream& operator<<(std::ostream& stream, const Matrix<T, rows, cols>& matrix)
{

  for (uint8_t row = 1; row <= rows; ++row)
  {
    if (row == 1)
      stream << "[";
    else
      stream << " ";
    for (uint8_t col = 1; col <= cols; ++col)
    {
      stream << std::setw(15) << std::setprecision(7) << matrix(row, col);
      if (row == rows && col == cols)
        stream << "         ]\n";
    }
    stream << "\n";
  }
  stream << "\n";
  return stream;
}
#endif

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator+(const T& scalar, const Matrix<T, rows, cols>& matrix)
{
  Matrix<T, rows, cols> sum;
  for (uint8_t row = 1; row <= rows; ++row)
    for (uint8_t col = 1; col <= cols; ++col)
      sum(row, col) = matrix(row, col) + scalar;
  return sum;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator+(const Matrix<T, rows, cols>& matrix, const T& scalar)
{
  Matrix<T, rows, cols> sum;
  for (uint8_t row = 1; row <= rows; ++row)
    for (uint8_t col = 1; col <= cols; ++col)
      sum(row, col) = matrix(row, col) + scalar;
  return sum;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator+(const Matrix<T, rows, cols>& matrix1,
                                const Matrix<T, rows, cols>& matrix2)
{
  Matrix<T, rows, cols> sum;
  for (uint8_t row; row <= rows; ++row)
    for (uint8_t col; col <= cols; ++col)
      sum(row, col) = matrix1(row, col) + matrix2(row, col);
  return sum;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator-(const T& scalar, const Matrix<T, rows, cols>& matrix)
{
  Matrix<T, rows, cols> result;
  for (uint8_t row = 1; row <= rows; ++row)
    for (uint8_t col = 1; col <= cols; ++col)
      result(row, col) = scalar - matrix(row, col);
  return result;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator-(const Matrix<T, rows, cols>& matrix, const T& scalar)
{
  Matrix<T, rows, cols> diff;
  for (uint8_t row = 1; row <= rows; ++row)
    for (uint8_t col = 1; col <= cols; ++col)
      diff(row, col) = matrix(row, col) - scalar;
  return diff;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator-(const Matrix<T, rows, cols>& matrix1,
                                const Matrix<T, rows, cols>& matrix2)
{
  Matrix<T, rows, cols> diff;
  for (uint8_t row = 1; row <= rows; ++row)
    for (uint8_t col = 1; col <= cols; ++col)
      diff(row, col) = matrix1(row, col) - matrix2(row, col);
  return diff;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator-(const Matrix<T, rows, cols>& matrix)
{
  Matrix<T, rows, cols> opposite;
  for (uint8_t row = 1; row <= rows; ++row)
    for (uint8_t col = 1; col <= cols; ++col)
      opposite(row, col) = -matrix(row, col);
  return opposite;
}

template <typename T, uint8_t rows1, uint8_t dim_common, uint8_t cols2>
Matrix<T, rows1, cols2> operator*(const Matrix<T, rows1, dim_common>& matrix1,
                                  const Matrix<T, dim_common, cols2>& matrix2)
{
  Matrix<T, rows1, cols2> prod;
  for (uint8_t row = 1; row <= rows1; ++row)
  {
    for (uint8_t col = 1; col <= cols2; ++col)
    {
      T sum = 0;
      for (uint8_t j = 1; j <= dim_common; j++)
        sum += matrix1(row, j) * matrix2(j, col);
      prod(row, col) = sum;
    }
  }
  return prod;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator*(const Matrix<T, rows, 1>& vvect,
                                const Matrix<T, 1, cols>& hvect)
{
  Matrix<T, rows, cols> prod;
  for (uint8_t row = 1; row <= rows; ++row)
  {
    for (uint8_t col = 1; col <= cols; ++col)
    {
      prod(row, col) = vvect(row, 1) * hvect(1, col);
    }
  }
  return prod;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator*(const T& scalar, const Matrix<T, rows, cols>& matrix)
{
  Matrix<T, rows, cols> prod;
  for (uint8_t row = 1; row <= rows; ++row)
    for (uint8_t col = 1; col <= cols; ++col)
      prod(row, col) = matrix(row, col) * scalar;
  return prod;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator*(const Matrix<T, rows, cols>& matrix, const T& scalar)
{
  Matrix<T, rows, cols> prod;
  for (uint8_t row = 1; row <= rows; ++row)
    for (uint8_t col = 1; col <= cols; ++col)
      prod(row, col) = matrix(row, col) * scalar;
  return prod;
}

template <uint8_t rows, typename T>
Matrix<T, rows, 1> operator*(const Matrix<T, rows, 1>& vvect1,
                             const Matrix<T, rows, 1>& vvect2)
{
  Matrix<T, rows, 1> prod;
  for (uint8_t row = 1; row <= rows; ++row)
    prod(row, 1) = vvect1(row, 1) * vvect2(row, 1);
  return prod;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator/(const Matrix<T, rows, cols>& matrix, const T& scalar)
{
  Matrix<T, rows, cols> result;
  for (uint8_t row = 1; row <= rows; ++row)
    for (uint8_t col = 1; col <= cols; ++col)
      result(row, col) = matrix(row, col) / scalar;
  return result;
}

template <typename T, uint8_t rows, uint8_t cols1, uint8_t cols2>
Matrix<T, rows, cols1 + cols2> HorzCat(const Matrix<T, rows, cols1>& matrix_lx,
                                       const Matrix<T, rows, cols2>& matrix_rx)
{
  Matrix<T, rows, cols1 + cols2> result;

  for (uint8_t row = 1; row <= rows; ++row)
  {
    uint8_t col;
    for (col = 1; col <= cols1; ++col)
      result(row, col) = matrix_lx(row, col);
    for (col = 1; col <= cols2; ++col)
      result(row, col + cols1) = matrix_rx(row, col);
  }
  return result;
}

template <typename T, uint8_t rows1, uint8_t rows2, uint8_t cols>
Matrix<T, rows1 + rows2, cols> VertCat(const Matrix<T, rows1, cols>& matrix_up,
                                       const Matrix<T, rows2, cols>& matrix_down)
{
  Matrix<T, rows1 + rows2, cols> result;
  for (uint8_t col = 1; col <= cols; ++col)
  {
    uint8_t row;
    for (row = 1; row <= rows1; ++row)
      result(row, col) = matrix_up(row, col);
    for (row = 1; row <= rows2; ++row)
      result(row + rows1, col) = matrix_down(row, col);
  }
  return result;
}

template <typename T, uint8_t dim>
T Dot(Matrix<T, dim, 1>& vvect1, Matrix<T, dim, 1>& vvect2)
{
  T result = 0;
  for (uint8_t i = 1; i <= dim; ++i)
  {
    result += vvect1(i, 1) * vvect2(i, 1);
  }
  return result;
}

template <typename T, uint8_t dim>
T Dot(const Matrix<T, 1, dim>& hvect, const Matrix<T, dim, 1>& vvect)
{
  T result = 0;
  for (uint8_t i = 1; i <= dim; ++i)
  {
    result += hvect(1, i) * vvect(i, 1);
  }
  return result;
}

template <typename T, uint8_t dim> T Norm(const Matrix<T, dim, 1>& vvect)
{
  T result = 0;
  for (uint8_t i = 1; i <= dim; i++)
    result += vvect(i, 1) * vvect(i, 1);
  return sqrt(result);
}

template <typename T, uint8_t dim> T Norm(const Matrix<T, 1, dim>& hvect)
{
  T result = 0;
  for (uint8_t i = 1; i <= dim; i++)
    result += hvect(1, i) * hvect(1, i);
  return sqrt(result);
}

template <typename T>
Matrix<T, 3, 1> Cross(const Matrix<T, 3, 1>& vvect3d1, const Matrix<T, 3, 1>& vvect3d2)
{
  return Anti(vvect3d1) * vvect3d2;
}

template <typename T> Matrix<T, 3, 3> Anti(const Matrix<T, 3, 1>& vvect3d)
{
  Matrix<T, 3, 3> result;
  result(1, 1) = 0;
  result(2, 2) = 0;
  result(3, 3) = 0;
  result(1, 2) = -vvect3d(3, 1);
  result(2, 1) = vvect3d(3, 1);
  result(1, 3) = vvect3d(2, 1);
  result(3, 1) = -vvect3d(2, 1);
  result(2, 3) = -vvect3d(1, 1);
  result(3, 2) = vvect3d(1, 1);
  return result;
}

} //  end namespace grabnum