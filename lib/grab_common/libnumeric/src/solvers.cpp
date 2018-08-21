/**
 * @file solvers.cpp
 * @author Edoardo Idà, Simone Comari
 * @date 21 Aug 2018
 * @brief File containing definitions and implementation of solvers.
 */

#include "solvers.h"

namespace grabnum
{

template <typename T, uint8_t dim>
VectorX<T, dim> Linsolve(const Matrix<T, dim, dim>& mat, const VectorX<T, dim>& vect)
{
  for (uint8_t i = 1; i < dim; ++i)
  {
    uint8_t max_idx = i;
    T max_val = mat(i, i);
    for (uint8_t j = i + 1; j <= dim; ++j)
    {
      if (fabs(mat(j, i)) > fabs(max_val))
      {
        max_val = mat(j, i);
        max_idx = j;
      }
    }
    mat.SwapRow(i, max_idx);
    std::swap(vect(i), vect(max_idx));
    for (uint8_t j = i + 1; j <= dim; ++j)
    {
      T m = mat(j, i) / mat(i, i);
      vect(j) -= vect(i) * m;
      for (uint8_t k = i; k <= dim; ++k)
      {
        mat(j, k) -= mat(i, k) * m;
      }
    }
  }
  return LinsolveUp(mat, vect);
}

template <typename T, uint8_t dim>
void Linsolve(const Matrix<T, dim, dim>& mat, const VectorX<T, dim>& vect,
              VectorX<T, dim>& result)
{
  for (uint8_t i = 1; i < dim; ++i)
  {
    uint8_t max_idx = i;
    T max_val = mat(i, i);
    for (uint8_t j = i + 1; j <= dim; ++j)
    {
      if (fabs(mat(j, i)) > fabs(max_val))
      {
        max_val = mat(j, i);
        max_idx = j;
      }
    }
    mat.SwapRow(i, max_idx);
    std::swap(vect(i), vect(max_idx));
    for (uint8_t j = i + 1; j <= dim; ++j)
    {
      T m = mat(j, i) / mat(i, i);
      vect(j) -= vect(i) * m;
      for (uint8_t k = i; k <= dim; ++k)
      {
        mat(j, k) -= mat(i, k) * m;
      }
    }
  }
  LinsolveUp(mat, vect, result);
}

template <typename T, uint8_t dim>
VectorX<T, dim> LinsolveUp(const Matrix<T, dim, dim>& mat, const VectorX<T, dim>& vect)
{
  VectorX<T, dim> result;
  result = vect;
  result(dim) /= mat(dim, dim);
  for (uint8_t j = dim - 1; j > 0; j--)
  {
    for (uint8_t k = j + 1; k <= dim; ++k)
    {
      result(j) -= mat(j, k) * result(k);
    }
    result(j) /= mat(j, j);
  }

  return result;
}

template <typename T, uint8_t dim>
void LinsolveUp(const Matrix<T, dim, dim>& mat, const VectorX<T, dim>& vect,
                VectorX<T, dim>& result)
{
  result = vect;
  result(dim) /= mat(dim, dim);
  for (uint8_t j = dim - 1; j > 0; j--)
  {
    for (uint8_t k = j + 1; k <= dim; ++k)
    {
      result(j) -= mat(j, k) * result(k);
    }
    result(j) /= mat(j, j);
  }
}

template <typename T, uint8_t dim>
int NonLinsolveJacobian(VectorX<T, dim>& solution,
                        void (*fP)(VectorX<T, dim>&, Matrix<T, dim, dim>&,
                                   const VectorX<T, dim>&))
{
  static const double ftol = 1e-9;
  static const double xtol = 1e-7;
  uint8_t iter = 0;
  uint8_t nmax = 100;
  double err = 1.0;
  double cond = 0.0;
  VectorX<T, dim> F;
  VectorX<T, dim> s;
  Matrix<T, dim, dim> J;

  fP(F, J, solution);

  while (iter < nmax && Norm(F) > ftol && err > cond)
  {
    iter++;
    Linsolve(J, s, F);
    solution -= s;
    fP(F, J, solution);
    err = Norm(s);
    cond = xtol * (1 + Norm(solution));
  }

  return iter;
}

template <typename T, uint8_t dim>
int fsolveB(VectorX<T, dim>& solution,
            void (*fP)(VectorX<T, dim>&, const VectorX<T, dim>&))
{
  static const double ftol = 1e-9;
  static const double xtol = 1e-9;
  uint8_t iter = 0;
  uint8_t nmax = 100;
  double err = 1.0;
  double cond = 0.0;
  VectorX<T, dim> F;
  VectorX<T, dim> s;
  Matrix<T, dim, dim> B;

  fP(F, solution);
  B.Identity();
  B -= (F * F.Transpose()) / (Dot(F, F));

  while (iter < nmax && Norm(F) > ftol && err > cond)
  {
    iter++;
    Linsolve(B, s, F);
    solution -= s;
    fP(F, solution);
    B += (F * s.Transpose()) / (Dot(s, s));
    err = Norm(s);
    cond = xtol * (1 + Norm(solution));
  }

  return iter;
}

} // end namespace grabnum
