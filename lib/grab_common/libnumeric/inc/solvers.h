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
 * Solve a linear system in matrix form.
 *
 * @param[in] mat A @a mxm square matrix.
 * @param[in] vect A m-dimensional vector.
 * @return A m-dimensional vector with the solution.
 */
template <typename T, uint8_t dim>
VectorX<T, dim> Linsolve(const Matrix<T, dim, dim>& mat, const VectorX<T, dim>& vect);

/**
 * Solve a linear system in matrix form.
 *
 * @param[in] mat A @a mxm square matrix.
 * @param[in] vect A m-dimensional vector.
 * @param[out] result A m-dimensional vector with the solution.
 */
template <typename T, uint8_t dim>
void Linsolve(const Matrix<T, dim, dim>& mat, const VectorX<T, dim>& vect,
              VectorX<T, dim>& result);

/**
 * Solve a linear system in matrix form when the coefficient matrix is upper-triangular.
 *
 * @param[in] mat A @a mxm square upper-triangular matrix.
 * @param[in] vect A m-dimensional vector.
 * @return A m-dimensional vector with the solution.
 */
template <typename T, uint8_t dim>
VectorX<T, dim> LinsolveUp(const Matrix<T, dim, dim>& mat, const VectorX<T, dim>& vect);

/**
 * Solve a linear system in matrix form when the coefficient matrix is upper-triangular.
 *
 * @param[in] mat A @a mxm square upper-triangular matrix.
 * @param[in] vect A m-dimensional vector.
 * @param[out] result A m-dimensional vector with the solution.
 */
template <typename T, uint8_t dim>
void LinsolveUp(const Matrix<T, dim, dim>& mat, const VectorX<T, dim>& vect,
                VectorX<T, dim>& result);

/**
 * Solve a non-linear system??
 *
 * @todo this.
 * @return A scalar.
 */
template <typename T, uint8_t dim>
int NonLinsolveJacobian(VectorX<T, dim>& solution,
                        void (*fP)(VectorX<T, dim>&, Matrix<T, dim, dim>&,
                                   const VectorX<T, dim>&));

/**
 * Solve a non-linear system??
 *
 * @todo this.
 * @return A scalar.
 */
template <typename T, uint8_t dim>
int fsolveB(VectorX<T, dim>& solution,
            void (*fP)(VectorX<T, dim>&, const VectorX<T, dim>&));

} // end namespace grabnum

#endif // GRABCOMMON_LIBNUMERIC_SOLVERS_H
