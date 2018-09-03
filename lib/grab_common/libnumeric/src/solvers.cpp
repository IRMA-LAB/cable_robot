/**
 * @file solvers.cpp
 * @author Edoardo Idà, Simone Comari
 * @date 21 Aug 2018
 * @brief File containing definitions and implementation of solvers.
 */

#include "solvers.h"

namespace grabnum
{

namespace solvers
{

template <typename T, uint8_t dim>
VectorX<T, dim> Linsolve(const Matrix<T, dim, dim>& _mat, const VectorX<T, dim>& _vect)
{
  static_assert(std::is_floating_point<T>::value, "ERROR: invalid type in LinSolve()!");

  Matrix<T, dim, dim>& mat(_mat);
  VectorX<T, dim>& vect(_vect);
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
    vect.SwapRow(i, max_idx);
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
void Linsolve(const Matrix<T, dim, dim>& _mat, const VectorX<T, dim>& _vect,
              VectorX<T, dim>& result)
{
  static_assert(std::is_floating_point<T>::value, "ERROR: invalid type in LinSolve()!");

  Matrix<T, dim, dim>& mat(_mat);
  VectorX<T, dim>& vect(_vect);
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
    vect.SwapRow(i, max_idx);
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
  static_assert(std::is_floating_point<T>::value, "ERROR: invalid type in LinSolveUp()!");

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
  static_assert(std::is_floating_point<T>::value, "ERROR: invalid type in LinSolveUp()!");

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
int NonLinsolveJacobian(void (*fun_ptr)(VectorX<T, dim>&, Matrix<T, dim, dim>&,
                const VectorX<T, dim>&), VectorX<T, dim>& solution,
                const uint8_t nmax /*= 100*/)
{
  static_assert(std::is_floating_point<T>::value, "ERROR: invalid type in NonLinsolveJacobian()!");

  static const T ftol = 1e-9;
  static const T xtol = 1e-7;
  uint8_t iter = 0;
  T err = 1.0;
  T cond = 0.0;
  VectorX<T, dim> F;
  VectorX<T, dim> s;
  Matrix<T, dim, dim> J;

  fun_ptr(F, J, solution);

  while (iter < nmax && Norm(F) > ftol && err > cond)
  {
    iter++;
    Linsolve(J, s, F);
    solution -= s;
    fun_ptr(F, J, solution);
    err = Norm(s);
    cond = xtol * (1 + Norm(solution));
  }

  return iter;
}

template <typename T, uint8_t dim>
int fsolveB(void (*fun_ptr)(VectorX<T, dim>&, const VectorX<T, dim>&),
            VectorX<T, dim>& solution, const uint8_t nmax /*= 100*/)
{
  static_assert(std::is_floating_point<T>::value, "ERROR: invalid type in fsolveB()!");

  static const T ftol = 1e-9;
  static const T xtol = 1e-9;
  uint8_t iter = 0;
  T err = 1.0;
  T cond = 0.0;
  VectorX<T, dim> F;
  VectorX<T, dim> s;
  Matrix<T, dim, dim> B;

  fun_ptr(F, solution);
  B.Identity();
  B -= (F * F.Transpose()) / (Dot(F, F));

  while (iter < nmax && Norm(F) > ftol && err > cond)
  {
    iter++;
    Linsolve(B, s, F);
    solution -= s;
    fun_ptr(F, solution);
    B += (F * s.Transpose()) / (Dot(s, s));
    err = Norm(s);
    cond = xtol * (1 + Norm(solution));
  }

  return iter;
}

template <typename T, uint8_t dim, size_t t_steps>
void RKSolver(void (*fun_ptr)(const T, const VectorX<T, dim>, VectorX<T, dim>),
              const VectorX<T, t_steps>& time, const VectorX<T, dim>& y0,
              Matrix<T, dim, t_steps>& sol)
{
  static constexpr uint8_t rk_dim = 6;
  static const VectorX<T, rk_dim> c(
    std::vector<T>{0., 0.25, 0.375, 12. / 13., 1., 0.5});
  static const VectorX<T, rk_dim> b(
    std::vector<T>{16. / 135., 0., 6656. / 12825., 28561. / 56430., -0.18, 2. / 5.});
  static const Matrix<T, rk_dim, rk_dim> rk_mat(std::vector<T>{
                                                  0., 0., 0., 0., 0., 0.,
                                                  0.25, 0., 0., 0., 0., 0,
                                                  0.09375, 0, 28125, 0., 0., 0., 0.,
                                                  1932. / 2197., -7200. / 2197., 7296. / 2197., 0., 0., 0.,
                                                  439. / 216., -8., 3680. / 513., -845. / 4104., 0, 0,
                                                  -8. / 27., 2., -3544. / 2565., 1859. / 4104., -0.275, 0.});

  // Initialize
  Matrix<T, dim, rk_dim> K;
  VectorX<T, dim> f, s, col;
  T h = time(2) - time(1);
  sol.SetZero();
  sol.SetCol(1, y0);

  // Solve
  for (size_t i = 2; i <= t_steps; ++i)
  {
    f.SetZero();
    for (uint8_t j = 1; j <= rk_dim; ++j)
    {
      s.SetZero();
      for (uint8_t k = 1; k <= j - 1; ++k)
        {
          s += rk_mat(j, k) * K.getCol(k);
        }
      fun_ptr(time(i - 1) + h * c(j), sol.GetCol(i - 1) + h * s, col);
      K.SetCol(j, col);
      f += b(j) * col;
    }
    sol.SetCol(i, sol.GetCol(i - 1) + h * f);
  }
}

} // end namespace solvers

} // end namespace grabnum
