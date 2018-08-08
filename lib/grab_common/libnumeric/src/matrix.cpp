#include "matrix.h"

namespace grabnum
{

///////////////////////////////////////////////////////////////////////////////
/// Constructors
///////////////////////////////////////////////////////////////////////////////

template <typename T, uint8_t rows, uint8_t cols> inline Matrix<T, rows, cols>::Matrix()
{
}

template <typename T, uint8_t rows, uint8_t cols>
inline Matrix<T, rows, cols>::Matrix(T scalar)
{
  if (scalar == 0)
    SetZero();
  if (scalar == 1)
    SetIdentity();
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>::Matrix(const T* values)
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements[row][col] = values[row * cols + col];
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
      if (elements[row][col] != other.elements[row][col])
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
      if (elements[col][row] != other.elements[col][row])
        return true;
    }
  return false;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::operator+=(const T& scalar)
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements[row][col] += scalar;
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::
operator+=(const Matrix<T, rows, cols>& other)
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements[row][col] += other.elements[row][col];
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::operator-=(const T& scalar)
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements[row][col] -= scalar;
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::
operator-=(const Matrix<T, rows, cols>& other)
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements[row][col] -= other.elements[row][col];
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::operator*=(const T& scalar)
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements[row][col] *= scalar;
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::operator/=(const T& scalar)
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements[row][col] /= scalar;
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
      elements[row - 1][col - 1] = other.elements[row - start_row][col - start_col];
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetCol(uint8_t cl,
                                                     const Matrix<T, rows, 1>& matrix1d)
{
  for (uint8_t i = 0; i < rows; i++)
  {
    elements[i][cl - 1] = matrix1d.elements[i][0];
  }
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetCol(uint8_t cl, const T* vect)
{
  for (uint8_t i = 0; i < rows; i++)
  {
    elements[i][cl - 1] = vect[i];
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
      elements[row][col] = other.elements[row + start_row - 1][col + start_col - 1];
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetIdentity()
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
    {
      if (row == col)
        elements[row][col] = static_cast<T>(1);
      else
        elements[row][col] = static_cast<T>(0);
    }
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetRow(uint8_t rw,
                                                     const Matrix<T, 1, cols>& matrix1d)
{
  for (uint8_t i = 0; i < cols; i++)
  {
    elements[rw - 1][i] = matrix1d.elements[0][i];
  }
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetRow(uint8_t rw, const T* vect)
{
  for (uint8_t i = 0; i < cols; i++)
  {
    elements[rw - 1][i] = vect[i];
  }
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetZero()
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements[row][col] = static_cast<T>(0);
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::Fill(const T* values)
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements[row][col] = values[row * cols + col];
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
      transpose.elements[col][row] = elements[row][col];
  return transpose;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SwapRow(uint8_t row1, uint8_t row2)
{
  T temp;
  for (uint8_t i = 0; i < cols; i++)
  {
    temp = elements[row1 - 1][i];
    elements[row1 - 1][i] = elements[row2 - 1][i];
    elements[row1 - 1][i] = temp;
  }
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SwapCol(uint8_t col1, uint8_t col2)
{
  T temp;
  for (uint8_t i = 0; i < rows; i++)
  {
    temp = elements[i][col1 - 1];
    elements[i][col1 - 1] = elements[i][col2 - 1];
    elements[i][col2 - 1] = temp;
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
    row_vect.elements[0][col] = elements[row - 1][col];
  return row_vect;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, 1> Matrix<T, rows, cols>::GetCol(uint8_t col)
{
  Matrix<T, rows, 1> col_vect;
  for (uint8_t row = 0; row < rows; ++row)
    col_vect.elements[row][0] = elements[row][col - 1];
  return col_vect;
}

} //  end namespace grabnum
