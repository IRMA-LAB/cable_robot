/**
 * @file solvers.h
 * @author Edoardo Idà, Simone Comari
 * @date 21 Aug 2018
 * @brief File containing numeric solver to be included in the GRAB numeric library.
 */

#ifndef GRABCOMMON_LIBNUMERIC_SOLVERS_H
#define GRABCOMMON_LIBNUMERIC_SOLVERS_H

#include "matrix.h"

/**
 * Namespace for GRAB numeric library.
 */
namespace grabnum
{

/**
 * Namespace for GRAB numeric solvers.
 */
namespace solvers
{

/**
 * Solve a @f$m \times m@f$ linear system in matrix form.
 *
 * A generic linear system of _m_ equations with _n_unknowns can be written in matrix form
 *as
 * @f[
 * \mathbf{A}\mathbf{x} = \mathbf{b}
 * @f]
 * being @f$\mathbf{A} \in \mathbb{R}^{m \times n}@f$ the matrix of coefficients of the
 * system, @f$\mathbf{x} \in \mathbb{R}^n@f$ the column vector of unknowns and
 * @f$\mathbf{b} \in \mathbb{R}^m@f$ the column vector of constant terms.
 * This functions finds solutions @f$\mathbf{x}@f$ for the case @f$m=n@f$.
 * @param[in] _mat A @f$m \times m@f$ square matrix of coefficients @f$\mathbf{A}@f$.
 * @param[in] _vect A @f$m@f$-dimensional vector of constant terms @f$\mathbf{b}@f$.
 * @return A @f$m@f$-dimensional vector with the solution @f$\mathbf{x}@f$.
 */
template <typename T, uint8_t dim>
VectorX<T, dim> Linsolve(const Matrix<T, dim, dim>& _mat, const VectorX<T, dim>& _vect);

/**
 * Solve a @f$m \times m@f$ linear system in matrix form.
 *
 * @param[in] _mat A @f$m \times m@f$ square matrix of coefficients @f$\mathbf{A}@f$.
 * @param[in] _vect A @f$m@f$-dimensional vector of constant terms @f$\mathbf{b}@f$.
 * @param[out] result A @f$m@f$-dimensional vector with the solution @f$\mathbf{x}@f$.
 * @see Linsolve()
 */
template <typename T, uint8_t dim>
void Linsolve(const Matrix<T, dim, dim>& _mat, const VectorX<T, dim>& _vect,
              VectorX<T, dim>& result);

/**
 * Solve a @f$m \times m@f$ linear system in matrix form where the coefficients matrix is
 * upper-triangular.
 *
 * @param[in] mat A @f$m \times m@f$ square upper-triangular matrix.
 * @param[in] vect A @f$m@f$-dimensional vector of constant terms.
 * @return A @f$m@f$-dimensional vector with the solution.
 * @see Linsolve()
 */
template <typename T, uint8_t dim>
VectorX<T, dim> LinsolveUp(const Matrix<T, dim, dim>& mat, const VectorX<T, dim>& vect);

/**
 * Solve a @f$m \times m@f$ linear system in matrix form where the coefficient matrix is
 * upper-triangular.
 *
 * @param[in] mat A @f$m \times m@f$ square upper-triangular matrix.
 * @param[in] vect A @f$m@f$-dimensional vector of constant terms.
 * @param[out] result A @f$m@f$-dimensional vector with the solution.
 * @see LinsolveUp()
 */
template <typename T, uint8_t dim>
void LinsolveUp(const Matrix<T, dim, dim>& mat, const VectorX<T, dim>& vect,
                VectorX<T, dim>& result);

/**
 * Solve a non-linear system??
 *
 * @param[in] fun_ptr Pointer to function.
 * @param[out] solution Solution vector.
 * @param[in] nmax (Optional) Maximum number of iterations. Default is 100.
 * @todo this.
 * @return A scalar with the number of iterations.
 */
template <typename T, uint8_t dim>
int NonLinsolveJacobian(void (*fun_ptr)(VectorX<T, dim>&, Matrix<T, dim, dim>&,
                                        const VectorX<T, dim>&),
                        VectorX<T, dim>& solution, const uint8_t nmax = 100);

/**
 * Solve a non-linear system??
 *
 * @param[in] fun_ptr Pointer to function.
 * @param[out] solution Solution vector.
 * @param[in] nmax (Optional) Maximum number of iterations. Default is 100.
 * @todo this.
 * @return A scalar with the number of iterations.
 */
template <typename T, uint8_t dim>
int fsolveB(void (*fun_ptr)(VectorX<T, dim>&, const VectorX<T, dim>&),
            VectorX<T, dim>& solution, const uint8_t nmax = 100);

/**
 * @brief _Runge–Kutta–Fehlberg method_ for the numerical solution of ODEs.
 *
 * The _Runge–Kutta–Fehlberg method_ (or _Fehlberg method_) is an algorithm in numerical
 * analysis for the numerical solution of ordinary differential equations. It is a method
 *of order
 * @f$O(h^4)@f$ with an error estimator of order @f$O(h^5)@f$.
 * @param[in] fun_ptr Pointer to differential equation of type
 * @f$\dot{\mathbf{y}} = f(t, \mathbf{y}), \mathbf{y} \in \mathbb{R}^m@f$. The arguments
 * of such function @f$f@f$ are (_time instant_ @f$t@f$ [s], _input vector_ @f$\mathbf{y}@f$,
 * _output vector_ @f$\dot{\mathbf{y}}@f$).
 * @param[in] time _n_-dimensional time vector with time step @f$h = t_k - t_{k-1}@f$ [s].
 * @param[in] y0 Values of @f$\mathbf{y}@f$ at initial time @f$t_0@f$, i.e.
 * @f$\mathbf{y}_0@f$.
 * @param[out] sol @f$m \times n@f$ solution matrix, where _i-th_ column represents the
 * solution of the problem at instant @f$t_i@f$, i.e. @f$\mathbf{y}_i@f$.
 */
template <typename T, uint8_t dim, size_t t_steps>
void RKSolver(void (*fun_ptr)(const T, const VectorX<T, dim>, VectorX<T, dim>),
              const VectorX<T, t_steps>& time, const VectorX<T, dim>& y0,
              Matrix<T, dim, t_steps>& sol);

} // end namespace solvers

} // end namespace grabnum

#endif // GRABCOMMON_LIBNUMERIC_SOLVERS_H
