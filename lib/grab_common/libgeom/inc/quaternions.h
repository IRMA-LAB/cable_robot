/**
 * @file quaternions.h
 * @author Edoardo Idà, Simone Comari
 * @date 06 Sep 2018
 * @brief File containing quaternions utilities to be included in the GRAB geometric
 * library.
 */

#ifndef GRABCOMMON_LIBGEOM_QUATERNIONS_H
#define GRABCOMMON_LIBGEOM_QUATERNIONS_H

#include "matrix.h"

namespace grabgeom
{

/**
*@brief The Quaternion struct
*/
struct Quaternion
{
  double w; /**< w component */
  double x; /**< x component  */
  double y; /**< y component */
  double z; /**< z component  */

  /**
   * @brief Quaternion
   */
  Quaternion() { Quaternion(0., 0., 0., 0.); }
  /**
   * @brief Quaternion
   * @param _w
   * @param _x
   * @param _y
   * @param _z
   */
  Quaternion(const double _w, const double _x, const double _y, const double _z)
  {
    w = _w;
    x = _x;
    y = _y;
    z = _z;
  }
  /**
   * @brief Quaternion
   * @param vect
   */
  Quaternion(const grabnum::VectorXd<4>& vect)
  {
    w = vect(1);
    x = vect(2);
    y = vect(3);
    z = vect(4);
  }
  /**
   * @brief Quaternion
   * @param _w
   * @param _v
   */
  Quaternion(const double _w, const grabnum::Vector3d& _v)
  {
    w = _w;
    x = _v(1);
    y = _v(2);
    z = _v(3);
  }

  /**
   * @brief operator ==
   * @param other
   * @return
   */
  bool operator==(const Quaternion& other) const { return v() == other.v(); }

  /**
   * @brief operator !=
   * @param other
   * @return
   */
  bool operator!=(const Quaternion& other) const { return v() != other.v(); }

  /**
   * @brief operator +=
   * @param other
   * @return
   */
  Quaternion& operator+=(const Quaternion& other)
  {
    w += other.w;
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }

  /**
   * @brief operator +=
   * @param scalar
   * @return
   */
  Quaternion& operator+=(const double scalar)
  {
    w += scalar;
    x += scalar;
    y += scalar;
    z += scalar;
    return *this;
  }

  /**
   * @brief operator -=
   * @param other
   * @return
   */
  Quaternion& operator-=(const Quaternion& other)
  {
    w -= other.w;
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
  }

  /**
   * @brief operator -=
   * @param scalar
   * @return
   */
  Quaternion& operator-=(const double scalar)
  {
    w -= scalar;
    x -= scalar;
    y -= scalar;
    z -= scalar;
    return *this;
  }

  /**
   * @brief operator *=
   * @param other
   * @return
   */
  Quaternion& operator*=(const Quaternion& other)
  {
    grabnum::Vector3d old_v = v();
    grabnum::Vector3d other_v = other.v();
    grabnum::Vector3d new_v =
      w * other_v + other.w * old_v + grabnum::Cross(old_v, other_v);
    w = w + other.w - grabnum::Dot(old_v, other_v);
    x = new_v(1);
    y = new_v(2);
    z = new_v(3);
    return *this;
  }

  /**
   * @brief operator *=
   * @param scalar
   * @return
   */
  Quaternion& operator*=(const double scalar)
  {
    w *= scalar;
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
  }

  /**
   * @brief operator /=
   * @param scalar
   * @return
   */
  Quaternion& operator/=(const double scalar)
  {
    w /= scalar;
    x /= scalar;
    y /= scalar;
    z /= scalar;
    return *this;
  }

  /**
   * @brief operator /
   * @param scalar
   * @return
   */
  Quaternion operator/(const double scalar) const
  {
    Quaternion res;
    res.w = w / scalar;
    res.x = x / scalar;
    res.y = y / scalar;
    res.z = z / scalar;
    return res;
  }

  /**
   * @brief v
   * @return
   */
  grabnum::Vector3d v() const
  {
    grabnum::Vector3d v;
    v(1) = x;
    v(2) = y;
    v(3) = z;
    return v;
  }

  /**
   * @brief q
   * @return
   */
  grabnum::VectorXd<4> q() const
  {
    grabnum::VectorXd<4> q;
    q(1) = w;
    q(2) = x;
    q(3) = y;
    q(4) = z;
    return q;
  }

  /**
   * @brief Identity
   * @return
   */
  Quaternion Identity() const { return Quaternion(1., 0., 0., 0.); }

  /**
   * @brief Conj
   * @return
   */
  Quaternion Conj() const { return Quaternion(w, -x, -y, -z); }

  /**
   * @brief Inv
   * @return
   */
  Quaternion Inv() const { return Conj() / Norm(); }

  /**
   * @brief Norm
   * @return
   */
  double Norm() const { return grabnum::Norm(q()); }

  /**
   * @brief Normalize
   * @return
   */
  Quaternion& Normalize()
  {
    *this /= Norm();
    return *this;
  }

  /**
   * @brief Normalized
   * @return
   */
  Quaternion Normalized() const { return *this / Norm(); }

  /**
   * @brief IsUnitary
   * @return
   */
  bool IsUnitary() const { return grabnum::IsClose(Norm(), 1.); }
};

/**
 * @brief operator +
 * @param lhs
 * @param rhs
 * @return
 */
Quaternion operator+(Quaternion lhs, const Quaternion& rhs) { return lhs += rhs; }

/**
 * @brief operator +
 * @param lhs
 * @param rhs
 * @return
 */
Quaternion operator+(Quaternion lhs, const double rhs) { return lhs += rhs; }

/**
 * @brief operator +
 * @param lhs
 * @param rhs
 * @return
 */
Quaternion operator+(const double lhs, Quaternion rhs) { return rhs += lhs; }

/**
 * @brief operator -
 * @param lhs
 * @param rhs
 * @return
 */
Quaternion operator-(Quaternion lhs, const Quaternion& rhs) { return lhs -= rhs; }

/**
 * @brief operator -
 * @param lhs
 * @param rhs
 * @return
 */
Quaternion operator-(Quaternion lhs, const double rhs) { return lhs -= rhs; }

/**
 * @brief operator -
 * @param lhs
 * @param rhs
 * @return
 */
Quaternion operator-(const double lhs, Quaternion rhs) { return rhs -= lhs; }

/**
 * @brief operator *
 * @param lhs
 * @param rhs
 * @return
 */
Quaternion operator*(Quaternion lhs, const Quaternion& rhs) { return lhs *= rhs; }

/**
 * @brief operator *
 * @param lhs
 * @param rhs
 * @return
 */
Quaternion operator*(Quaternion lhs, const double rhs) { return lhs *= rhs; }

/**
 * @brief operator *
 * @param lhs
 * @param rhs
 * @return
 */
Quaternion operator*(const double lhs, Quaternion rhs) { return rhs *= lhs; }

/**
 * @brief QuatConjugate
 * @param quat
 * @return
 */
Quaternion QuatConjugate(const Quaternion& quat) { return quat.Conj(); }

/**
 * @brief QuatInverse
 * @param quat
 * @return
 */
Quaternion QuatInverse(const Quaternion& quat) { return quat.Inv(); }

/**
 * @brief QuatNorm
 * @param quat
 * @return
 */
double QuatNorm(const Quaternion& quat) { return quat.Norm(); }

/**
 * @brief QuatNormalized
 * @param quat
 * @return
 */
Quaternion QuatNormalized(const Quaternion& quat) { return quat.Normalized(); }

/**
 * @brief Determines the rotation matrix corresponding to a given quaternion.
 *
 * @param[in] quaternion The orientation expressed by a unitary quaternion
 * @f$(q_w, q_x, q_y, q_z)@f$.
 * @return A 3x3 orthogonal matrix (double).
 */
grabnum::Matrix3d Quat2Rot(const Quaternion& quaternion);

/**
 * @brief Determines the quaternion corresponding to a given rotation matrix.
 * @param[in] rot_mat A rotation matrix.
 * @return A unitary quaternion @f$(q_w, q_x, q_y, q_z)@f$.
 */
Quaternion Rot2Quat(const grabnum::Matrix3d& rot_mat);

/**
 * @brief QuatRotate
 * @param quat
 * @param vect
 * @return
 */
grabnum::Vector3d QuatRotate(const Quaternion& quat, const grabnum::Vector3d& vect);

/**
 * @brief Transformation matrix @f$\mathbf{H}@f$ between the derivative of a quaternion
 * and angular velocity vector @f$\boldsymbol\omega@f$.
 *
 * @param[in] quaternion The orientation expressed by a unitary quaternion
 * @f$(q_w, q_x, q_y, q_z)@f$.
 * @return A 3x3 matrix (double).
 */
grabnum::MatrixXd<3, 4> HtfQuat(const Quaternion& quaternion);

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
grabnum::MatrixXd<3, 4> DHtfQuat(const Quaternion& quaternion_dot)
{
  return HtfQuat(quaternion_dot);
}

} // end namespace grabgeom

#endif // GRABCOMMON_LIBGEOM_QUATERNIONS_H
