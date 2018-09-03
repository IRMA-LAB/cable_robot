/**
 * @file rotations.h
 * @author Edoardo Idà, Simone Comari
 * @date 27 Aug 2018
 * @brief File containing rotation parametrizations to be included in the GRAB geometric
 * library.
 *
 * @note
 * ### Rotation interpretations
 * @note
 * A generic rotation matrix @f$^0\mathbf{R}_1@f$ can be interpreted in three different
 * ways:
 * 1. @f$^0\mathbf{R}_1@f$ describes the mutual orientation of two reference frames
 * @f$\mathcal{F}_0@f$ and @f$\mathcal{F}_1@f$; the columns of
 * @f$^0\mathbf{R}_1@f$ are the _direction cosines_ of the axes of @f$\mathcal{F}_1@f$
 * expressed in @f$\mathcal{F}_0@f$. Let @f$^0\mathbf{\hat{x}}_1@f$ be the _unit
 * vector_ representing @f$x@f$-axis of frame @f$\mathcal{F}_1@f$ expressed in frame
 * @f$\mathcal{F}_0@f$. Similar notation yields for @f$^0\mathbf{\hat{y}}_1@f$ and
 * @f$^0\mathbf{\hat{z}}_1@f$. Then it holds:
 * @f[
 *     ^0\mathbf{R}_1 =
 *                      \begin{bmatrix}
 *                          ^0\mathbf{\hat{x}}_1 & ^0\mathbf{\hat{y}}_1 &
 *^0\mathbf{\hat{z}}_1
 *                      \end{bmatrix}
 * @f]
 * 2. @f$^0\mathbf{R}_1@f$ defines the coordinate transformation between the
 * coordinates of a point expressed in @f$\mathcal{F}_1@f$ and in @f$\mathcal{F}_0@f$
 * (with common origin). Let @f$^0\mathbf{p}@f$ be a 3D point (sometimes refered as
 * _position vector_) expressed in frame @f$\mathcal{F}_1@f$. Then it holds:
 *      @f[^0\mathbf{p} = {^0\mathbf{R}_1} ~ ^1\mathbf{p}@f]
 * where @f$^0\mathbf{p}@f$ represents the same point expressed in frame
 * @f$\mathcal{F}_0@f$.
 * 3. @f$^0\mathbf{R}_1@f$ rotates a vector @f$^0\mathbf{v}_a@f$ to
 * @f$^0\mathbf{v}_b@f$ in a given reference frame @f$\mathcal{F}_0@f$:
 * @f[^0\mathbf{v}_b = {^0\mathbf{R}_1} ~ ^0\mathbf{v}_a@f]
 *
 * @note
 * ### Differential angular kinematics ###
 * @note
 * In general, given a triple of angles @f$\boldsymbol\epsilon@f$, i.e. a parametrization
 *of the
 * orientation of a rigid body, a transformation matrix
 * @f$\mathbf{H}(\boldsymbol{\epsilon})@f$ exists such that:
 * @f[ \boldsymbol{\omega} = \mathbf{H}(\boldsymbol\epsilon)\dot{\boldsymbol\epsilon} @f]
 * being @f$\boldsymbol\omega@f$ an angular velocity vector expressed in a third frame
 * @f$\mathcal{F}_2@f$ with origin coincident with @f$\mathcal{F}_0@f$, the base frame,
 * and axes parallel to @f$\mathcal{F}_1@f$, the body-fixed frame. The velocity vector
 * @f$\boldsymbol\omega@f$ is placed in the origin, and its direction coincides with the
 * instantaneous rotation axis of the rigid body.
 * @n The form of @f$\mathbf{H}@f$ specifically depends on the parametrization used.
 */

#ifndef GRABCOMMON_LIBGEOM_ROTATIONS_H
#define GRABCOMMON_LIBGEOM_ROTATIONS_H

#include "matrix.h"

namespace grabgeom
{

/**
 * @brief Elementary rotation around @f$X@f$-axis.
 *
 * Given two frames @f$\mathcal{F}_0@f$ and @f$\mathcal{F}_1@f$ with common origin,
 * @f$\mathcal{F}_1@f$ is obtained with a rotation of an angle @f$\theta@f$ about the
 * @f$\mathbf{x}_0@f$ axis of @f$\mathcal{F}_0@f$:
 * @f[
 *    ^0\mathbf{R}_1 = \begin{bmatrix}
 *                                      1 & 0 & 0 \\
 *                                      0 & \cos(\theta) & -\sin(\theta) \\
 *                                      1 & \sin(\theta) & \cos(\theta) \\
 *                                    \end{bmatrix}
 * @f]
 *
 * @param[in] angle Rotation angle in radians.
 * @return A 3x3 orthogonal matrix (double).
 */
grabnum::Matrix3d RotX(const double angle);

/**
 * @brief Elementary rotation around @f$Y@f$-axis.
 *
 * Given two frames @f$\mathcal{F}_0@f$ and @f$\mathcal{F}_1@f$ with common origin,
 * @f$\mathcal{F}_1@f$ is obtained with a rotation of an angle @f$\theta@f$ about the
 * @f$\mathbf{y}_0@f$ axis of @f$\mathcal{F}_0@f$:
 * @f[
 *    ^0\mathbf{R}_1 = \begin{bmatrix}
 *                                      \cos(\theta) & 0 & sin(\theta) \\
 *                                      0 & 1 & 0 \\
 *                                      -\sin(\theta) & 0 & \cos(\theta) \\
 *                                    \end{bmatrix}
 * @f]
 *
 * @param[in] angle Rotation angle in radians.
 * @return A 3x3 orthogonal matrix (double).
 */
grabnum::Matrix3d RotY(const double angle);

/**
 * @brief Elementary rotation around @f$Z@f$-axis.
 *
 * Given two frames @f$\mathcal{F}_0@f$ and @f$\mathcal{F}_1@f$ with common origin,
 * @f$\mathcal{F}_1@f$ is obtained with a rotation of an angle @f$\theta@f$ about the
 * @f$\mathbf{z}_0@f$ axis of @f$\mathcal{F}_0@f$:
 * @f[
 *    ^0\mathbf{R}_1 = \begin{bmatrix}
 *                                      \cos(\theta) & -sin(\theta) & 0 \\
 *                                      \sin(\theta) & \cos(\theta) & 0 \\
 *                                      0 & 0 & 1 \\
 *                                    \end{bmatrix}
 * @f]
 *
 * @param[in] angle Rotation angle in radians.
 * @return A 3x3 orthogonal matrix (double).
 */
grabnum::Matrix3d RotZ(const double angle);

/**
 * @brief Rotation matrix based on _Tait-Bryan_ angles convention and @f$X_1Y_2Z_3@f$
 *order.
 *
 * _Tait-Bryan_ angles @f$(\alpha,\beta,\gamma)@f$ represent three rotations, applied
 * sequentially about axes @f$\mathbf{x}_0, \mathbf{y}_1,\mathbf{z}_2@f$ of the _current_
 * frame @f$\mathcal{F}_0, \mathcal{F}_1, \mathcal{F}_2@f$.
 *
 * Consider a base frame @f$\mathcal{F}_0@f$. By applying the three rotations we have:
 * - A frame @f$\mathcal{F}_1@f$ obtained with the rotation @f$\alpha@f$ about
 * @f$\mathbf{x}_0@f$
 * - A frame @f$\mathcal{F}_2@f$ obtained from @f$\mathcal{F}_1@f$ with the rotation
 * @f$\beta@f$ about @f$\mathbf{y}_1@f$
 * - A frame @f$\mathcal{F}_3@f$ obtaine from @f$\mathcal{F}_2@f$ with the rotation
 * @f$\gamma@f$ about @f$\mathbf{z}_2@f$
 * .
 * By composing the three rotations, the total rotation from @f$\mathcal{F}_0@f$ to
 * @f$\mathcal{F}_3@f$ is:
 * @f[
 * ^0\mathbf{R}_3 = \mathbf{R}_{xyz}(\alpha,\beta,\gamma) = \mathbf{R}_{x_0}(\alpha)
 * \mathbf{R}_{y_1}(\beta) \mathbf{R}_{z_2}(\gamma)
 * @f]
 *
 * @param[in] alpha [rad] Rotation angle about @f$x_0@f$-axis.
 * @param[in] beta [rad] Rotation angle about @f$y_1@f$-axis.
 * @param[in] gamma [rad] Rotation angle about @f$z_2@f$-axis.
 * @return A 3x3 orthogonal matrix (double).
 */
grabnum::Matrix3d RotXYZ(const double alpha, const double beta, const double gamma);
/**
 * @brief Rotation matrix based on _Tait-Bryan_ angles convention and @f$X_1Y_2Z_3@f$
 *order.
 *
 * _Tait-Bryan_ angles @f$(\alpha,\beta,\gamma)@f$ represent three rotations, applied
 * sequentially about axes @f$\mathbf{x}_0, \mathbf{y}_1,\mathbf{z}_2@f$ of the _current_
 * frame @f$\mathcal{F}_0, \mathcal{F}_1, \mathcal{F}_2@f$.
 *
 * Consider a base frame @f$\mathcal{F}_0@f$. By applying the three rotations we have:
 * - A frame @f$\mathcal{F}_1@f$ obtained with the rotation @f$\alpha@f$ about
 * @f$\mathbf{x}_0@f$
 * - A frame @f$\mathcal{F}_2@f$ obtained from @f$\mathcal{F}_1@f$ with the rotation
 * @f$\beta@f$ about @f$\mathbf{y}_1@f$
 * - A frame @f$\mathcal{F}_3@f$ obtaine from @f$\mathcal{F}_2@f$ with the rotation
 * @f$\gamma@f$ about @f$\mathbf{z}_2@f$
 * .
 * By composing the three rotations, the total rotation from @f$\mathcal{F}_0@f$ to
 * @f$\mathcal{F}_3@f$ is:
 * @f[
 * ^0\mathbf{R}_3 = \mathbf{R}_{xyz}(\alpha,\beta,\gamma) = \mathbf{R}_{x_0}(\alpha)
 * \mathbf{R}_{y_1}(\beta) \mathbf{R}_{z_2}(\gamma)
 * @f]
 *
 * @param[in] angles [rad] _Tait-Bryan_ angles @f$(\alpha,\beta,\gamma)@f$ vector.
 * @return A 3x3 orthogonal matrix (double).
 */
grabnum::Matrix3d RotXYZ(const grabnum::Vector3d& angles)
{
  return RotXYZ(angles(1), angles(2), angles(3));
}

/**
 * @brief Rotation matrix based on _Roll, Pitch, Yaw_ angles convention (from aviation).
 *
 * _Roll, pitch, yaw_ angles @f$(\phi,\theta,\psi)@f$ represent three consecutive
 *rotations
 * about the axes of the _base_ frame @f$\mathcal{F}_0@f$:
 * - _roll_ is a counter-clockwise rotation of @f$\phi@f$ about the @f$x_0@f$-axis
 * - _pitch_ is a counter-clockwise rotation of @f$\theta@f$ about the @f$y_0@f$-axis
 * - _yaw_ is a counter-clockwise rotation of @f$\psi@f$ about the @f$z_0@f$-axis
 * .
 * By composing the three rotations, the total rotation from @f$\mathcal{F}_0@f$ to
 * @f$\mathcal{F}_3@f$ is:
 * @f[
 * ^0\mathbf{R}_3 = \mathbf{R}_{RPY}(\phi,\theta,\psi) = \mathbf{R}_{z_0}(\psi)
 * \mathbf{R}_{y_0}(\theta) \mathbf{R}_{x_0}(\phi)
 * @f]
 *
 * @param[in] roll [rad] Rolling angle (about @f$x_0@f$-axis).
 * @param[in] pitch [rad] Pitching angle (about @f$y_0@f$-axis).
 * @param[in] yaw [rad] Yawing angle (about @f$z_0@f$-axis).
 * @return A 3x3 orthogonal matrix (double).
 */
grabnum::Matrix3d RotRPY(const double roll, const double pitch, const double yaw);
/**
 * @brief Rotation matrix based on _Roll, Pitch, Yaw_ angles convention (from aviation).
 *
 * _Roll, pitch, yaw_ angles @f$(\phi,\theta,\psi)@f$ represent three consecutive
 *rotations
 * about the axes of the _base_ frame @f$\mathcal{F}_0@f$:
 * - _roll_ is a counter-clockwise rotation of @f$\phi@f$ about the @f$x_0@f$-axis
 * - _pitch_ is a counter-clockwise rotation of @f$\theta@f$ about the @f$y_0@f$-axis
 * - _yaw_ is a counter-clockwise rotation of @f$\psi@f$ about the @f$z_0@f$-axis
 * .
 * By composing the three rotations, the total rotation from @f$\mathcal{F}_0@f$ to
 * @f$\mathcal{F}_3@f$ is:
 * @f[
 * ^0\mathbf{R}_3 = \mathbf{R}_{RPY}(\phi,\theta,\psi) = \mathbf{R}_{z_0}(\psi)
 * \mathbf{R}_{y_0}(\theta) \mathbf{R}_{x_0}(\phi)
 * @f]
 *
 * @param[in] rpy [rad]  _Roll, pitch, yaw_ angles @f$(\phi,\theta,\psi)@f$ vector.
 * @return A 3x3 orthogonal matrix (double).
 */
grabnum::Matrix3d RotRPY(const grabnum::Vector3d& rpy)
{
  return RotRPY(rpy(1), rpy(2), rpy(3));
}

/**
 * @brief Rotation matrix based on _Euler_ angles convention and @f$Z_1Y_2Z_3@f$ order.
 *
 * _Euler_ angles @f$(\alpha,\beta,\gamma)@f$ represent three rotations, applied
 * sequentially about axes @f$\mathbf{z}_0, \mathbf{y}_1,\mathbf{z}_2@f$ of the _current_
 * frame @f$\mathcal{F}_0, \mathcal{F}_1, \mathcal{F}_2@f$.
 *
 * Consider a base frame @f$\mathcal{F}_0@f$. By applying the three rotations we have:
 * - A frame @f$\mathcal{F}_1@f$ obtained with the rotation @f$\alpha@f$ about
 * @f$\mathbf{z}_0@f$
 * - A frame @f$\mathcal{F}_2@f$ obtained from @f$\mathcal{F}_1@f$ with the rotation
 * @f$\beta@f$ about @f$\mathbf{y}_1@f$
 * - A frame @f$\mathcal{F}_3@f$ obtaine from @f$\mathcal{F}_2@f$ with the rotation
 * @f$\gamma@f$ about @f$\mathbf{z}_2@f$
 * .
 * By composing the three rotations, the total rotation from @f$\mathcal{F}_0@f$ to
 * @f$\mathcal{F}_3@f$ is:
 * @f[
 * ^0\mathbf{R}_3 = \mathbf{R}_{zyz}(\alpha,\beta,\gamma) = \mathbf{R}_{z_0}(\alpha)
 * \mathbf{R}_{y_1}(\beta) \mathbf{R}_{z_2}(\gamma)
 * @f]
 *
 * @param[in] alpha [rad] Rotation angle about @f$z_0@f$-axis.
 * @param[in] beta [rad] Rotation angle about @f$y_1@f$-axis.
 * @param[in] gamma [rad] Rotation angle about @f$z_2@f$-axis.
 * @return A 3x3 orthogonal matrix (double).
 */
grabnum::Matrix3d RotZYZ(const double alpha, const double beta, const double gamma);
/**
 * @brief Rotation matrix based on _Euler_ angles convention and @f$Z_1Y_2Z_3@f$ order.
 *
 * _Euler_ angles @f$(\alpha,\beta,\gamma)@f$ represent three rotations, applied
 * sequentially about axes @f$\mathbf{z}_0, \mathbf{y}_1,\mathbf{z}_2@f$ of the _current_
 * frame @f$\mathcal{F}_0, \mathcal{F}_1, \mathcal{F}_2@f$.
 *
 * Consider a base frame @f$\mathcal{F}_0@f$. By applying the three rotations we have:
 * - A frame @f$\mathcal{F}_1@f$ obtained with the rotation @f$\alpha@f$ about
 * @f$\mathbf{z}_0@f$
 * - A frame @f$\mathcal{F}_2@f$ obtained from @f$\mathcal{F}_1@f$ with the rotation
 * @f$\beta@f$ about @f$\mathbf{y}_1@f$
 * - A frame @f$\mathcal{F}_3@f$ obtaine from @f$\mathcal{F}_2@f$ with the rotation
 * @f$\gamma@f$ about @f$\mathbf{z}_2@f$
 * .
 * By composing the three rotations, the total rotation from @f$\mathcal{F}_0@f$ to
 * @f$\mathcal{F}_3@f$ is:
 * @f[
 * ^0\mathbf{R}_3 = \mathbf{R}_{zyz}(\alpha,\beta,\gamma) = \mathbf{R}_{z_0}(\alpha)
 * \mathbf{R}_{y_1}(\beta) \mathbf{R}_{z_2}(\gamma)
 * @f]
 *
 * @param[in] angles [rad] _Euler_ angles @f$(\alpha,\beta,\gamma)@f$ vector.
 * @return A 3x3 orthogonal matrix (double).
 */
grabnum::Matrix3d RotZYZ(const grabnum::Vector3d& angles)
{
  return RotZYZ(angles(1), angles(2), angles(3));
}

/**
 * @brief Rotation matrix based on _tilt-and-torsion_ angle system.

 * _Tilt-and-torsion_ angle system is a variation of _Euler_ angles convention where:
 * - @f$\alpha = \phi @f$ is the _tilt-azimuth_ angle
 * - @f$\beta = \theta @f$ is the _tilt_ angle
 * - @f$\gamma = \tau -  \phi@f$, being @f$\tau@f$ the _torsion_ angle
 * .
 * Hence, the total rotation from @f$\mathcal{F}_0@f$ to @f$\mathcal{F}_3@f$ becomes:
 * @f[
 * ^0\mathbf{R}_3 = \mathbf{R}_{zyz}(\alpha,\beta,\gamma) =
 * \mathbf{R}_{zyz}(\phi,\theta,\tau -  \phi) =
 * \mathbf{R}_{z_0}(\phi) \mathbf{R}_{y_1}(\theta) \mathbf{R}_{z_2}(\tau -  \phi)
 * @f]
 *
 * @param[in] tilt_azimuth [rad] Tilt-azimuth angle (about @f$z_0@f$-axis).
 * @param[in] tilt [rad] Tilt angle (about @f$y_1@f$-axis).
 * @param[in] torsion [rad] Torsion angle (about @f$z_2@f$-axis).
 * @return A 3x3 orthogonal matrix (double).
 * @see RotZYZ()
 */
grabnum::Matrix3d RotTiltTorsion(const double tilt_azimuth, const double tilt,
                                 const double torsion);
/**
 * @brief Rotation matrix based on _tilt-and-torsion_ angle system.

 * _Tilt-and-torsion_ angle system is a variation of _Euler_ angles convention where:
 * - @f$\alpha = \phi @f$ is the _tilt-azimuth_ angle
 * - @f$\beta = \theta @f$ is the _tilt_ angle
 * - @f$\gamma = \tau -  \phi@f$, being @f$\tau@f$ the _torsion_ angle
 * .
 * Hence, the total rotation from @f$\mathcal{F}_0@f$ to @f$\mathcal{F}_3@f$ becomes:
 * @f[
 * ^0\mathbf{R}_3 = \mathbf{R}_{zyz}(\alpha,\beta,\gamma) =
 * \mathbf{R}_{zyz}(\phi,\theta,\tau -  \phi) =
 * \mathbf{R}_{z_0}(\phi) \mathbf{R}_{y_1}(\theta) \mathbf{R}_{z_2}(\tau -  \phi)
 * @f]
 *
 * @param[in] angles [rad] _Tilt-and-torsion_ angles @f$(\phi,\theta,\tau)@f$ vector.
 * @return A 3x3 orthogonal matrix (double).
 * @see RotZYZ()
 */
grabnum::Matrix3d RotTiltTorsion(const grabnum::Vector3d& angles)
{
  return RotTiltTorsion(angles(1), angles(2), angles(3));
}

/**
 * @brief Transformation matrix @f$\mathbf{H}@f$ between the derivative of Tait-Bryan
 *angles and angular
 * velocity vector @f$\boldsymbol\omega@f$.
 *
 * @param[in] alpha [rad] Rotation angle about @f$x_0@f$-axis.
 * @param[in] beta [rad] Rotation angle about @f$y_1@f$-axis.
 * @return A 3x3 matrix (double).
 */
grabnum::Matrix3d HtfXYZ(const double alpha, const double beta);
/**
 * @brief Transformation matrix @f$\mathbf{H}@f$ between the derivative of _Tait-Bryan_
 *angles and angular
 * velocity vector @f$\boldsymbol\omega@f$.
 *
 * @param[in] angles [rad] _Tait-Bryan_ angles @f$(\alpha,\beta,\gamma)@f$ vector.
 * @return A 3x3 matrix (double).
 */
grabnum::Matrix3d HtfXYZ(const grabnum::Vector3d& angles)
{
  return HtfXYZ(angles(1), angles(2));
}

/**
 * @brief Transformation matrix @f$\mathbf{H}@f$ between the derivative of
 * _Roll, Pitch, Yaw_ angles and angular velocity vector @f$\boldsymbol\omega@f$.
 *
 * @param[in] roll [rad] Rolling angle (about @f$x_0@f$-axis).
 * @param[in] pitch [rad] Pitching angle (about @f$y_0@f$-axis).
 * @return A 3x3 matrix (double).
 * @see RotRPY()
 */
grabnum::Matrix3d HtfRPY(const double roll, const double pitch);
/**
 * @brief Transformation matrix @f$\mathbf{H}@f$ between the derivative of
 * _Roll, Pitch, Yaw_ angles and angular velocity vector @f$\boldsymbol\omega@f$.
 *
 * @param[in] rpy [rad]  _Roll, pitch, yaw_ angles @f$(\phi,\theta,\psi)@f$ vector.
 * @return A 3x3 matrix (double).
 * @see HtfRPY()
 */
grabnum::Matrix3d HtfRPY(const grabnum::Vector3d& rpy)
{
  return HtfRPY(rpy(1), rpy(2));
}

/**
 * @brief Transformation matrix @f$\mathbf{H}@f$ between the derivative of
 * _tilt-and-torsion_ angles and angular velocity vector @f$\boldsymbol\omega@f$.
 *
 * @param[in] tilt_azimuth [rad] Tilt-azimuth angle (about @f$z_0@f$-axis).
 * @param[in] tilt [rad] Tilt angle (about @f$y_1@f$-axis).
 * @return A 3x3 matrix (double).
 */
grabnum::Matrix3d HtfTiltTorsion(const double tilt_azimuth, const double tilt);
/**
 * @brief Transformation matrix @f$\mathbf{H}@f$ between the derivative of
 * _tilt-and-torsion_ angles and angular velocity vector @f$\boldsymbol\omega@f$.
 *
 * @param[in] angles [rad] _Tilt-and-torsion_ angles @f$(\phi,\theta,\tau)@f$ vector.
 * @return A 3x3 matrix (double).
 */
grabnum::Matrix3d HtfTiltTorsion(const grabnum::Vector3d& angles)
{
  return HtfTiltTorsion(angles(1), angles(2));
}

/**
 * @brief Transformation matrix @f$\mathbf{H}@f$ between the derivative of _Euler_ angles
 * and angular velocity vector @f$\boldsymbol\omega@f$.
 *
 * @param[in] alpha [rad] Rotation angle about @f$z_0@f$-axis.
 * @param[in] beta [rad] Rotation angle about @f$y_1@f$-axis.
 * @return A 3x3 matrix (double).
 */
grabnum::Matrix3d HtfZYZ(const double alpha, const double beta);
/**
 * @brief Transformation matrix @f$\mathbf{H}@f$ between the derivative of _Euler_ angles
 * and angular velocity vector @f$\boldsymbol\omega@f$.
 *
 * @param[in] angles [rad] _Euler_ angles @f$(\alpha,\beta,\gamma)@f$ vector.
 * @return A 3x3 matrix (double).
 */
grabnum::Matrix3d HtfZYZ(const grabnum::Vector3d& angles)
{
  return HtfZYZ(angles(1), angles(2));
}

/**
 * @brief Time derivative of transformation matrix between the derivative of _Tait-Bryan_
 * angles and angular velocity vector @f$\boldsymbol\omega@f$.
 *
 * @f[
 * \mathbf{\dot{H}}(\boldsymbol\epsilon, \boldsymbol{\dot{\epsilon}}) =
 * \frac{d\mathbf{H}(\boldsymbol\epsilon)}{dt}
 * @f]
 *
 * @param[in] alpha [rad] Rotation angle about @f$x_0@f$-axis.
 * @param[in] beta [rad] Rotation angle about @f$y_1@f$-axis.
 * @param[in] alpha_dot [rad/s] Time derivative of rotation angle alpha about
 *@f$x_0@f$-axis.
 * @param[in] beta_dot [rad/s] Time derivative of rotation angle about @f$y_1@f$-axis.
 * @return A 3x3 matrix (double).
 * @see HtfXYZ()
 */
grabnum::Matrix3d DHtfXYZ(const double alpha, const double beta, const double alpha_dot,
                          const double beta_dot);
/**
 * @brief Time derivative of transformation matrix between the derivative of _Tait-Bryan_
 * angles and angular velocity vector @f$\boldsymbol\omega@f$.
 *
 * @f[
 * \mathbf{\dot{H}}(\boldsymbol\epsilon, \boldsymbol{\dot{\epsilon}}) =
 * \frac{d\mathbf{H}(\boldsymbol\epsilon)}{dt}
 * @f]
 *
 * @param[in] angles [rad] _Tait-Bryan_ angles @f$(\alpha,\beta,\gamma)@f$ vector.
 * @param[in] angles_dot [rad] _Tait-Bryan_ angles derivatives
 * @f$(\dot\alpha,\dot\beta,\dot\gamma)@f$ vector.
 * @return A 3x3 matrix (double).
 * @see HtfXYZ()
 */
grabnum::Matrix3d DHtfXYZ(const grabnum::Vector3d& angles,
                          const grabnum::Vector3d& angles_dot)
{
  return DHtfXYZ(angles(1), angles(2), angles_dot(1), angles_dot(2));
}

/**
 * @brief Time derivative of transformation matrix between the derivative of
 * _Roll, Pitch, Yaw_ angles and angular velocity vector @f$\boldsymbol\omega@f$.
 *
 * @f[
 * \mathbf{\dot{H}}(\boldsymbol\epsilon, \boldsymbol{\dot{\epsilon}}) =
 * \frac{d\mathbf{H}(\boldsymbol\epsilon)}{dt}
 * @f]
 *
 * @param[in] roll [rad] Rolling angle (about @f$x_0@f$-axis).
 * @param[in] pitch [rad] Pitching angle (about @f$y_0@f$-axis).
 * @param[in] roll_dot [rad/s] Time derivative of rolling angle (about @f$x_0@f$-axis).
 * @param[in] pitch_dot [rad/s] Time derivative of pitching angle (about @f$y_0@f$-axis).
 * @return A 3x3 matrix (double).
 * @see HtfRPY()
 */
grabnum::Matrix3d DHtfRPY(const double roll, const double pitch, const double roll_dot,
                          const double pitch_dot);
/**
 * @brief Time derivative of transformation matrix between the derivative of _Euler_
 * angles and angular velocity vector @f$\boldsymbol\omega@f$.
 *
 * @f[
 * \mathbf{\dot{H}}(\boldsymbol\epsilon, \boldsymbol{\dot{\epsilon}}) =
 * \frac{d\mathbf{H}(\boldsymbol\epsilon)}{dt}
 * @f]
 *
 * @param[in] rpy [rad]  _Roll, pitch, yaw_ angles @f$(\phi,\theta,\psi)@f$ vector.
 * @param[in] rpy_dot [rad] _Roll, pitch, yaw_ angles derivatives
 * @f$(\dot\phi,\dot\theta,\dot\psi)@f$ vector.
 * @return A 3x3 matrix (double).
 * @see HtfRPY()
 */
grabnum::Matrix3d DHtfRPY(const grabnum::Vector3d& rpy,
                          const grabnum::Vector3d& rpy_dot)
{
  return DHtfRPY(rpy(1), rpy(2), rpy_dot(1), rpy_dot(2));
}

/**
 * @brief Time derivative of transformation matrix between the derivative of _Euler_
 * angles and angular velocity vector @f$\boldsymbol\omega@f$.
 *
 * @f[
 * \mathbf{\dot{H}}(\boldsymbol\epsilon, \boldsymbol{\dot{\epsilon}}) =
 * \frac{d\mathbf{H}(\boldsymbol\epsilon)}{dt}
 * @f]
 *
 * @param[in] alpha [rad] Rotation angle about @f$z_0@f$-axis.
 * @param[in] beta [rad] Rotation angle about @f$y_1@f$-axis.
 * @param[in] alpha_dot [rad/s] Time derivative of rotation angle _alpha_ about
 *@f$z_0@f$-axis.
 * @param[in] beta_dot [rad/s] Time derivative of rotation angle _beta_ about @f$y_1@f$-axis.
 * @return A 3x3 matrix (double).
 * @see HtfXYZ()
 */
grabnum::Matrix3d DHtfZYZ(const double alpha, const double beta, const double alpha_dot,
                          const double beta_dot);
/**
 * @brief Time derivative of transformation matrix between the derivative of _Euler_
 * angles and angular velocity vector @f$\boldsymbol\omega@f$.
 *
 * @f[
 * \mathbf{\dot{H}}(\boldsymbol\epsilon, \boldsymbol{\dot{\epsilon}}) =
 * \frac{d\mathbf{H}(\boldsymbol\epsilon)}{dt}
 * @f]
 *
 * @param[in] angles [rad] _Euler_ angles @f$(\alpha,\beta,\gamma)@f$ vector.
 * @param[in] angles_dot [rad] _Euler_ angles derivatives
 * @f$(\dot\alpha,\dot\beta,\dot\gamma)@f$ vector.
 * @return A 3x3 matrix (double).
 * @see HtfXYZ()
 */
grabnum::Matrix3d DHtfZYZ(const grabnum::Vector3d& angles,
                          const grabnum::Vector3d& angles_dot)
{
  return DHtfZYZ(angles(1), angles(2), angles_dot(1), angles_dot(2));
}

/**
 * @brief Time derivative of transformation matrix between the derivative of
 *tilt-and-torsion
 * angles and angular velocity vector @f$\boldsymbol\omega@f$.
 *
 * @f[
 * \mathbf{\dot{H}}(\boldsymbol\epsilon, \boldsymbol{\dot{\epsilon}}) =
 * \frac{d\mathbf{H}(\boldsymbol\epsilon)}{dt}
 * @f]
 *
 * @param[in] tilt_azimuth [rad] Tilt-azimuth angle (about @f$z_0@f$-axis).
 * @param[in] tilt [rad] Tilt angle (about @f$y_1@f$-axis).
 * @param[in] tilt_azimuth_dot [rad/s] Time derivative of tilt-azimuth angle (about
 *@f$x_0@f$-axis).
 * @param[in] tilt_dot [rad/s] Time derivative of tilt angle (about @f$y_1@f$-axis).
 * @return A 3x3 matrix (double).
 * @see HtfTiltTorsion()
 */
grabnum::Matrix3d DHtfTiltTorsion(const double tilt_azimuth, const double tilt,
                                  const double tilt_azimuth_dot, const double tilt_dot);
/**
 * @brief Time derivative of transformation matrix between the derivative of
 *tilt-and-torsion
 * angles and angular velocity vector @f$\boldsymbol\omega@f$.
 *
 * @f[
 * \mathbf{\dot{H}}(\boldsymbol\epsilon, \boldsymbol{\dot{\epsilon}}) =
 * \frac{d\mathbf{H}(\boldsymbol\epsilon)}{dt}
 * @f]
 *
 * @param[in] angles [rad] _Tilt-and-torsion_ angles @f$(\phi,\theta,\tau)@f$ vector.
 * @param[in] angles_dot [rad] _Tilt-and-torsion_ angles derivatives
 * @f$(\dot\phi,\dot\theta,\dot\tau)@f$ vector.
 * @return A 3x3 matrix (double).
 * @see HtfTiltTorsion()
 */
grabnum::Matrix3d DHtfTiltTorsion(const grabnum::Vector3d& angles,
                                  const grabnum::Vector3d& angles_dot)
{
  return DHtfTiltTorsion(angles(1), angles(2), angles_dot(1), angles_dot(2));
}

} // end namespace grabgeom

#endif // GRABCOMMON_LIBGEOM_ROTATIONS_H
