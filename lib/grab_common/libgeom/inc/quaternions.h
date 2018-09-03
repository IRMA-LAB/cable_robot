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

} // end namespace grabgeom

#endif // GRABCOMMON_LIBGEOM_QUATERNIONS_H
