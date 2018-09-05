/**
 * @file quaternions.h
 * @author Edoardo Idà, Simone Comari
 * @date 03 Sep 2018
 * @brief File containing quaternions utilities to be included in the GRAB geometric
 * library.
 * @todo quaternion basic operations.
 */

#ifndef GRABCOMMON_LIBGEOM_QUATERNIONS_H
#define GRABCOMMON_LIBGEOM_QUATERNIONS_H

#include "matrix.h"

namespace grabgeom
{

using Quaternion = grabnum::VectorXd<4>;  /**< quaternion vector */

/**
 * @brief Determines the rotation matrix corresponding to a given quaternion.
 *
 * @param[in] quaternion The orientation expressed by a quaternion
 * @f$(q_w, q_x, q_y, q_z)@f$.
 * @return A 3x3 orthogonal matrix (double).
 */
grabnum::Matrix3d Quat2Rot(const Quaternion& quaternion);

/**
 * @brief Determines the quaternion corresponding to a given rotation matrix.
 * @param[in] rot_mat A rotation matrix.
 * @return A quaternion @f$(q_w, q_x, q_y, q_z)@f$.
 */
Quaternion Rot2Quat(const grabnum::Matrix3d& rot_mat);

/**
 * @brief Transformation matrix @f$\mathbf{H}@f$ between the derivative of a quaternion
 * and angular velocity vector @f$\boldsymbol\omega@f$.
 *
 * @param[in] quaternion The orientation expressed by a quaternion
 * @f$(q_w, q_x, q_y, q_z)@f$.
 * @return A 3x3 matrix (double).
 */
grabnum::MatrixXd<3,4> HtfQuat(const Quaternion& quaternion);

/**
 * @brief Time-derivative of transformation matrix @f$\mathbf{H}@f$ between the
 * derivative of a quaternion and angular velocity vector @f$\boldsymbol\omega@f$.
 *
 * @f[
 * \mathbf{\dot{H}}(\boldsymbol{\epsilon}_q, \boldsymbol{\dot{\epsilon}}_q) =
 * \frac{d\mathbf{H}(\boldsymbol{\epsilon}_q)}{dt}
 * @f]
 *
 * @param[in] quaternion_dot The orientation speed expressed by a quaternion
 * @f$\dot{\boldsymbol{\epsilon}}_q@f$.
 * @return A 3x3 matrix (double).
 */
grabnum::MatrixXd<3,4> DHtfQuat(const Quaternion& quaternion_dot)
{
  return HtfQuat(quaternion_dot);
}

} // end namespace grabgeom

#endif // GRABCOMMON_LIBGEOM_QUATERNIONS_H
